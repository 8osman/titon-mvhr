#include <cassert>
#include <cstdio>
#include <cstring>
#include "../relay_guard.h"
#include "../hrv_bus.h"
#include "../mini_json.h"
#include "../table_seq.h"
using namespace hrv;
static int fails=0;
#define CHECK(c) do{ if(!(c)){ printf("FAIL line %d: %s\n",__LINE__,#c); fails++; } }while(0)

static const ChannelLimits L[4]={{4000,10000},{3000,6000},{4000,10000},{1000,2000}};

void testGuard(){
  RelayGuard g(L,4,1,1000,500,200);
  uint32_t t=100000; g.begin(t);
  CHECK(g.request(1,true,0,t+10)==Result::BootHold);          // boot hold
  t+=1000;
  CHECK(g.request(1,true,0,t)==Result::Ok);                   // default 3000
  CHECK(g.isOn(1));
  CHECK(g.request(0,true,0,t+300)==Result::Interlock);        // one at a time
  CHECK(g.request(2,true,0,t+300)==Result::Interlock);
  CHECK(g.request(3,true,0,t+300)==Result::Ok);               // aux not interlocked
  CHECK(g.remainingMs(1,t+1000)==2000);
  CHECK((g.tick(t+2999)&2)==0);
  CHECK((g.tick(t+3000)&2));                                  // S2 timed out
  CHECK(!g.isOn(1));
  CHECK(g.request(1,true,0,t+3100)==Result::TooSoon);         // min off
  CHECK(g.request(1,true,99999,t+4000)==Result::Ok);          // clamped
  CHECK(g.remainingMs(1,t+4000)==6000);
  CHECK(g.request(1,false,0,t+4100)==Result::Ok && !g.isOn(1));
  // blocked
  g.setBlocked(0,true);
  CHECK(g.request(0,true,0,t+9000)==Result::Blocked);
  g.setBlocked(0,false);
  CHECK(g.request(0,true,0,t+9000)==Result::Ok);
  g.setBlocked(0,true);
  CHECK((g.tick(t+9001)&1) && !g.isOn(0));                    // blocked while on -> opens
  CHECK(g.request(9,true,0,t)==Result::BadChannel);
  // never all three even if asked
  RelayGuard g3(L,4,3,0,0,0); g3.begin(0);
  CHECK(g3.maxClosed()==2);
  CHECK(g3.request(0,true,0,10)==Result::Ok);
  CHECK(g3.request(1,true,0,10)==Result::Ok);
  CHECK(g3.request(2,true,0,10)==Result::Interlock);
  CHECK(g3.closedSwitches()==2);
  // wrap safety
  RelayGuard gw(L,4,1,0,0,0); uint32_t w=0xFFFFFF00u; gw.begin(w);
  CHECK(gw.request(0,true,1000,w+1)==Result::Ok);
  CHECK(gw.tick(w+500)==0);
  CHECK(gw.tick(w+1200)==1);                                  // crosses 2^32
  CHECK(g3.allOff(20)==3);
}
void testParse(){
  Frame f;
  CHECK(parseLine("3801+00000",10,f)&&f.kind==KIND_REQUEST&&!f.write&&f.addr==380&&f.value==0);
  CHECK(parseLine("3850+00002",10,f)&&f.write&&f.addr==385&&f.value==2);
  CHECK(parseLine("380+00123",9,f)&&f.kind==KIND_REPLY&&f.addr==380&&f.value==123);
  CHECK(parseLine("383-00050",9,f)&&f.value==-50);
  CHECK(!parseLine("FW0046-102",10,f));
  CHECK(!parseLine("38x1+00000",10,f));
  CHECK(!parseLine("3802+00000",10,f));
  CHECK(!parseLine("380+0012",8,f));
  // line assembler incl. junk without CRLF then a good frame
  LineAssembler a; uint32_t t=0; bool got=false;
  const char* junk="FW0046-102";
  for(const char*p=junk;*p;++p){ got|=a.feed(*p,t); t+=8; }
  CHECK(!got);
  t+=500;                                                      // gap resync
  const char* good="384+00066\r\n"; int n=0;
  for(const char*p=good;*p;++p){ if(a.feed(*p,t)) n++; t+=8; }
  CHECK(n==1 && strcmp(a.line(),"384+00066")==0 && a.junk()==1);
  CHECK(parseLine(a.line(),a.length(),f)&&f.value==66);
  // blank lines ignored
  CHECK(!a.feed('\r',t) && !a.feed('\n',t+1));
  // overlong
  LineAssembler b; t=0; for(int i=0;i<40;i++){ b.feed('1',t); t+=1; } CHECK(b.junk()>=1);
}
void testStatus(){
  StatusBits s=decodeStatus(66); CHECK(s.speed==2 && !s.switchClosed);
  s=decodeStatus(0x24); CHECK(s.speed==3 && s.switchClosed);
  s=decodeStatus(0x204); CHECK(s.speed==3 && s.overrun);
  s=decodeStatus(0x9008); CHECK(s.speed==4 && s.summerboost && s.bypass && !s.fault);
  s=decodeStatus(0x4840); CHECK(s.speed==0 && s.fault);
  s=decodeStatus(0x6800); CHECK(s.speed==0 && s.fault);
  s=decodeStatus(0x21); CHECK(s.speed==1 && s.switchClosed);
  s=decodeStatus(3); CHECK(s.speed==0);
  s=decodeStatus(257); CHECK(s.speed==1 && s.boostInhibit && !s.rhControl);
  s=decodeStatus(68); CHECK(s.speed==3 && s.rhControl && !s.boostInhibit);
  s=decodeStatus(66); CHECK(s.speed==2 && s.rhControl);
  s=decodeStatus(1); CHECK(!s.boostInhibit && !s.rhControl);
}
void testJson(){
  char b[256]; JsonOut j(b,sizeof b); j.begin();
  j.num("a",19.8,1,true); j.integer("b",5,false); j.boolean("c",true,true); j.str("d","x\"y"); j.str("e",nullptr); j.end();
  CHECK(strcmp(b,"{\"a\":19.8,\"b\":null,\"c\":true,\"d\":\"x_y\",\"e\":null}")==0);
  char s[10]; JsonOut k(s,sizeof s); k.begin(); k.str("long","0123456789"); k.end(); CHECK(!k.ok());
}
void testSettings(){
  SettingsStore st; CHECK(st.set(330,3)); CHECK(!st.set(330,3)); CHECK(st.set(330,5));
  int32_t v; CHECK(st.get(330,v)&&v==5); CHECK(!st.get(331,v));
  CHECK(isSettingAddr(305)&&isSettingAddr(241)&&!isSettingAddr(380));
}
void testTx(){
  char b[16];
  CHECK(buildRequest(b,385,true,2)==12 && strcmp(b,"3850+00002\r\n")==0);
  CHECK(buildRequest(b,70,false,0)==12 && strcmp(b,"0701+00000\r\n")==0);
  CHECK(buildRequest(b,306,true,99999)==12 && strcmp(b,"3060+99999\r\n")==0);
  CHECK(buildRequest(b,1000,false,0)==0 && buildRequest(b,5,true,100000)==0 && buildRequest(b,5,true,-1)==0);
  Frame f; buildRequest(b,385,true,2); CHECK(parseLine(b,10,f)&&f.write&&f.addr==385&&f.value==2);
  CHECK(isWritableAddr(385,false)&&isWritableAddr(160,false)&&isWritableAddr(161,false)&&isWritableAddr(300,false)&&isWritableAddr(307,false));
  CHECK(!isWritableAddr(330,false)&&!isWritableAddr(241,false)&&!isWritableAddr(380,false)&&!isWritableAddr(308,false)&&!isWritableAddr(124,false));
  CHECK(isWritableAddr(330,true)&&isWritableAddr(332,true)&&!isWritableAddr(380,true)&&!isWritableAddr(70,true));
  RelayGuard g(L,4,1,0,0,0); g.begin(1000);
  g.request(3,true,2000,2000); CHECK(g.elapsedOnMs(3,2500)==500 && g.elapsedOnMs(0,2500)==0);
}
void testStaleClock(){
  RelayGuard g(L,4,1,0,0,0); g.begin(1000);
  CHECK(g.request(3,true,0,5000)==Result::Ok);       // command handled at t=5000
  CHECK(g.tick(4990)==0 && g.isOn(3));                // loop still holding an older clock reading
  CHECK(g.remainingMs(3,4990)==1000 && g.elapsedOnMs(3,4990)==0);
  CHECK((g.tick(6000)&8) && !g.isOn(3));              // times out normally (aux default 1000)
}

static void testScan(){
  hrv::Plan p; hrv::buildScanPlan(p);
  CHECK(p.n == hrv::kNumScan && p.n <= hrv::kMaxSteps);
  for(int i=0;i<p.n;i++) CHECK(p.s[i].kind==hrv::STEP_READ);   // never writes
  CHECK(hrv::kScanAddrs[0]==40 && hrv::kScanAddrs[hrv::kNumScan-1]==239);
  for(int i=0;i<hrv::kNumScan;i++) CHECK(hrv::kScanAddrs[i]!=68);   // never probe 068 (factory reset)
}
static void testCfgWrite(){
  hrv::Plan p; hrv::buildCfgWritePlan(p, 250, 5);
  CHECK(p.n==4 && p.preReads==1);
  CHECK(p.s[0].kind==hrv::STEP_READ && p.s[1].kind==hrv::STEP_WRITE_CONST && p.s[2].kind==hrv::STEP_WRITE_CONST && p.s[3].kind==hrv::STEP_READ);
  CHECK(p.s[0].addr==250 && p.s[1].addr==250 && p.s[3].addr==250 && p.s[1].value==5);
  CHECK(p.s[2].addr==161 && p.s[2].value==0);       // menu-closed marker, as the Aura-T sends on exit
  hrv::Plan q; hrv::buildCfgWritePlan(q, 250, 5, false);
  CHECK(q.n==3 && q.s[1].kind==hrv::STEP_WRITE_CONST && q.s[2].kind==hrv::STEP_READ);
  // proven registers are writable, unknown ones are not
  const int ok[] = {231,232,241,250,253,290,345};
  for(int a: ok){ const hrv::CfgReg* r=hrv::findCfg(a); CHECK(r && r->writable && hrv::isWritableAddr(a,true) && !hrv::isWritableAddr(a,false)); }
  CHECK(!hrv::findCfg(240)->writable && !hrv::findCfg(28)->writable);
  CHECK(!hrv::isWritableAddr(240,true) && !hrv::isWritableAddr(28,true));
  CHECK(hrv::findCfg(290)->invBool && !hrv::findCfg(250)->invBool);
  // manual ranges
  CHECK(hrv::findCfg(241)->lo==300 && hrv::findCfg(241)->hi==1000 && hrv::findCfg(241)->scale==10);
  CHECK(hrv::findCfg(231)->lo==170 && hrv::findCfg(231)->hi==350);
  CHECK(hrv::findCfg(232)->lo==100 && hrv::findCfg(232)->hi==200);
  CHECK(hrv::findCfg(250)->hi==60 && hrv::findCfg(253)->lo==0 && hrv::findCfg(345)->lo==1 && hrv::findCfg(345)->hi==24);
  CHECK(!hrv::findCfg(380));
  // every writable setting must be recognised as a stored setting so verify reads are kept
  for(int i=0;i<hrv::kNumCfgRegs;i++) CHECK(hrv::isSettingAddr(hrv::kCfgRegs[i].addr));
}
static void testSettingsPlan(){
  hrv::Plan p; hrv::buildSettingsReadPlan(p);
  CHECK(p.n == hrv::kNumCfgRegs + 3);
  for(int i=0;i<p.n;i++) CHECK(p.s[i].kind==hrv::STEP_READ);   // never writes
  CHECK(p.s[0].addr==28 && p.s[p.n-1].addr==332);
  hrv::SettingsStore st;
  for(int i=0;i<hrv::kNumCfgRegs;i++) CHECK(hrv::isSettingAddr(hrv::kCfgRegs[i].addr));  // replies get stored
  CHECK(!hrv::isWritableAddr(240,true) && !hrv::isWritableAddr(28,true));            // unknown registers never writable
  CHECK(hrv::isWritableAddr(250,true) && !hrv::isWritableAddr(250,false));
  (void)st;
}
static void testTable(){
  hrv::Plan p; hrv::buildReadPlan(p);
  CHECK(p.n==8 && p.preReads==8);
  for(int i=0;i<8;i++) CHECK(p.s[i].kind==hrv::STEP_READ && p.s[i].addr==300+i);
  hrv::buildSavePlan(p);
  CHECK(p.n==26);
  CHECK(p.s[8].kind==hrv::STEP_WRITE_CONST && p.s[8].addr==160 && p.s[8].value==1);
  for(int i=0;i<8;i++) CHECK(p.s[9+i].kind==hrv::STEP_WRITE_TABLE && p.s[9+i].addr==300+i);
  CHECK(p.s[17].kind==hrv::STEP_WRITE_CONST && p.s[17].addr==161 && p.s[17].value==0);
  for(int i=0;i<8;i++) CHECK(p.s[18+i].kind==hrv::STEP_READ);
  // every table write is in the allow-list; no write before the 8 pre-reads
  for(int i=0;i<p.n;i++){ if(p.s[i].kind!=hrv::STEP_READ) CHECK(i>=8 && hrv::isWritableAddr(p.s[i].addr,false)); }
  CHECK(hrv::speedSupplyAddr(1)==306 && hrv::speedSupplyAddr(2)==302 && hrv::speedSupplyAddr(3)==304 && hrv::speedSupplyAddr(4)==300 && hrv::speedSupplyAddr(5)==0);
  CHECK(hrv::tableValueOk(140,140,1000) && !hrv::tableValueOk(139,140,1000) && !hrv::tableValueOk(1001,140,1000));
}
int main(){ testScan(); testCfgWrite(); testSettingsPlan(); testTable(); testStaleClock(); testTx(); testGuard(); testParse(); testStatus(); testJson(); testSettings();
  printf(fails?"%d FAILED\n":"all passed\n",fails); return fails?1:0; }

#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <string>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define SERIAL_8N1 0x800001c
#define WL_CONNECTED 3
typedef uint8_t byte;
struct String : std::string {
  String(){} String(const char*s):std::string(s){} String(const std::string&s):std::string(s){}
  String(int v):std::string(std::to_string(v)){}
  const char* c_str() const {return std::string::c_str();}
  String substring(unsigned i) const {return String(this->substr(i));}
  bool startsWith(const String&p) const {return rfind(p,0)==0;}
  bool endsWith(const char*s) const {size_t n=strlen(s);return size()>=n&&compare(size()-n,n,s)==0;}
  long toInt() const {return atol(c_str());}
  String operator+(const char*s) const {return String(std::string(*this)+s);}
  String operator+(const String&s) const {return String(std::string(*this)+std::string(s));}
};
inline String operator+(const char*a,const String&b){return String(std::string(a)+std::string(b));}
uint32_t millis(); void delay(uint32_t); void pinMode(int,int); void digitalWrite(int,int); int digitalRead(int);
struct HardwareSerial { void begin(unsigned long,uint32_t=0,int=-1,int=-1); int available(); int read(); size_t write(const uint8_t*,size_t); void flush();
  void println(const char*); void printf(const char*,...) __attribute__((format(printf,2,3))); };
extern HardwareSerial Serial, Serial2;

#pragma once
#include "Arduino.h"
#include "WiFi.h"
struct PubSubClient { PubSubClient(WiFiClient&); void setServer(const char*,int); void setCallback(void(*)(char*,byte*,unsigned int));
 bool setBufferSize(int); void setSocketTimeout(int); bool connected(); bool connect(const char*,const char*,const char*,const char*,int,bool,const char*);
 bool publish(const char*,const char*,bool=false); bool subscribe(const char*); bool loop(); };

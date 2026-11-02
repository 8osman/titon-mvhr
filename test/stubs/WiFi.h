#pragma once
#include "Arduino.h"
#define WIFI_STA 1
struct WiFiClient {};
struct IPAddress { String toString() const; };
struct WiFiClass { void mode(int); void setHostname(const char*); void setAutoReconnect(bool); void begin(const char*,const char*); int status(); IPAddress localIP(); };
extern WiFiClass WiFi;

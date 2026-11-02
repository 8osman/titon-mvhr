#pragma once
#include <functional>
struct OTAClass { void setHostname(const char*); void setPassword(const char*); void onStart(std::function<void()>); void begin(); void handle(); };
extern OTAClass ArduinoOTA;

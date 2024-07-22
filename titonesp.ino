// titon Digit controller for esp32 (should work with also other titon - mvhr)
// Based upon
// (c) Toni Korhonen 2020
// https://www.creatingsmarthome.com/?p=73

#ifdef ESP32
#include <WiFi.h>
#include <ESPmDNS.h>
#else
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#endif

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "titonesp.h"
#include "titon.h"

#define JSON_BUFFER_LENGTH 2048
#define DEBUG false // default value for debug

#define titonESP_VERSION "1.0.0" // this version

// Callbacks
void mqttCallback(char* topic, byte* payload, unsigned int payloadLength);

WiFiClient wifiClient;
PubSubClient client(mqtt_server, mqtt_port, mqttCallback, wifiClient);
titon tn(DEBUG); 

unsigned long lastUpdated = 0;
bool debug = DEBUG;

void setup() {
  wifiConnect();
  mqttConnect();

  // Setup OTA
  initOTA();

  tn.setPacketCallback(packetDebug);
  tn.setStatusChangedCallback(statusChanged);
  tn.setDebugPrintCallback(debugPrint);
  tn.setTemperatureChangedCallback(temperatureChanged);
  
  tn.connect(&Serial);

  client.setCallback(mqttCallback);

  if (debug) {
    client.publish(titon_debug_topic, "Setup done.");
  }
}

void loop() {

  // loop tn messages
  tn.loop();

  // check that we are connected
  if (!client.loop()) {
    mqttConnect();
  }

#ifndef ESP32
  MDNS.update();
#endif

  ArduinoOTA.handle();
}

void initOTA() {
  //Serial.println("Start OTA Listener");
  ArduinoOTA.setHostname(client_id);
  ArduinoOTA.setPassword(ota_password);
  ArduinoOTA.begin();
}

void wifiConnect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    // wait 500ms
    delay(500);
  }

  WiFi.mode(WIFI_STA);
}

void mqttConnect() {
  if (!!!client.connected()) {
    int count = 20;
    while (count-- > 0 && !!!client.connect(client_id, mqtt_username, mqtt_password)) {
      delay(500);
    }

    if (client.subscribe(titon_set_topic)) {
      // OK
      if (debug) {
        client.publish(titon_debug_topic, "Subscribed to titon set topic.");
      }
    } else {
      // FAIL
      if (debug) {
        client.publish(titon_debug_topic, "Failed to subscribe to titon set topic.");
      }
    }
  }
}

void publishAllData() {
  publishState();
  publishTemperatures();
}

void mqttCallback(char* topic, byte * payload, unsigned int length) {
  if (strcmp (titon_set_topic, topic) == 0) {
    handleUpdate(payload);
  }
}

void handleUpdate(byte * payload) {
  DynamicJsonDocument d(JSON_BUFFER_LENGTH);
  DeserializationError error = deserializeJson(d, (char*)payload);

  if (error) {
    if (debug) {
      client.publish(titon_debug_topic, error.c_str());
    }
    return;
  }

  if (d.containsKey("DEBUG")) {
    boolean debg = d["DEBUG"];
    debug = debg;
    tn.setDebug(debug);
  }

  if (d.containsKey("mode")) {
    String m = d["mode"];

    if (m == "FAN") {
      // Fan only
      if (!tn.isOn()) {
        tn.setOn();
      }
      tn.setHeatingModeOff();
    } else if (m == "HEAT") {
      // Set heat mode
      if (!tn.isOn()) {
        tn.setOn();
      }
      tn.setHeatingModeOn();
    }
   
  }

  // I've disabled possibility to turn off the ventilation
  // If you wish to have such feature, feel free to implement :-)

  // Speed
  if (d.containsKey("speed")) {
    int speed = d["speed"];
    tn.setFanSpeed(speed);
  }

  // Heat target
  if (d.containsKey("heat_target")) {
    int ht = d["heat_target"];
    tn.setHeatingTarget(ht);
  }

  // Activate boost/fireplace
  if (d.containsKey("activate_switch")) {
    tn.setSwitchOn();
  }
}

// State
void publishState() {
  DynamicJsonDocument root(JSON_BUFFER_LENGTH);

  // Mode
  root["mode"] = !tn.isOn() ? "OFF" : (tn.isHeatingMode() ? "HEAT" : "FAN");

  // Boolean values
  root["heating"] = tn.isHeating();
  root["on"] = tn.isOn();
  root["fault"] = tn.isFault();
  root["rh_mode"] = tn.isRhMode();
  root["service_needed"] = tn.isServiceNeeded();
  
  root["summer_mode"] = tn.isSummerMode();
  // root["error_relay"] = tn.isErrorRelay();
  root["motor_in"] = !tn.isMotorIn();
  root["motor_out"] = !tn.isMotorOut();
  root["front_heating"] = tn.isFrontHeating();
 
  // Int values
  root["speed"] = tn.getFanSpeed();
  root["default_fan_speed"] = tn.getDefaultFanSpeed();
  root["service_period"] = tn.getServicePeriod();
  root["service_counter"] = tn.getServiceCounter();
  root["heat_target"] = tn.getHeatingTarget();  

  root["switch_active"] = tn.isSwitchActive();

  root["titonesp_sw_version"] = titonESP_VERSION;

  if (tn.getSwitchType() != NOT_SET) {
    root["switch_type"] = tn.getSwitchType() == 1 ? "boost" : "fireplace";
  }

  
  String mqttOutput;
  serializeJson(root, mqttOutput);
  client.beginPublish(titon_state_topic, mqttOutput.length(), true);
  client.print(mqttOutput);
  client.endPublish();
}

void publishTemperatures() {
  DynamicJsonDocument root(JSON_BUFFER_LENGTH);

  root["temp_outside"] = tn.getOutsideTemp();
  root["temp_inside"] = tn.getInsideTemp();
  root["temp_incoming"] = tn.getIncomingTemp();
  root["temp_exhaust"] = tn.getExhaustTemp();

  // Publish only if RH values are something else than not set
  if(tn.getRh1() != NOT_SET) {
    root["rh_1"] = tn.getRh1();
  }

  if(tn.getRh2() != NOT_SET) {
    root["rh_2"] = tn.getRh2();
  }

  if(tn.getCO2() != NOT_SET) {
    root["co2"] = tn.getCO2();
  }

  String mqttOutput;
  serializeJson(root, mqttOutput);
  client.beginPublish(titon_temp_topic, mqttOutput.length(), true);
  client.print(mqttOutput);
  client.endPublish();
}

void statusChanged() {
  publishState();
}

void temperatureChanged() {
  publishTemperatures();
}

void debugPrint(String message) {
  if (debug) {
    // publish to debug topic
    client.publish(titon_debug_topic, message.c_str());
  }
}

void packetDebug(byte* packet, unsigned int length, char* packetDirection) {
  if (debug) {
    String message;
    for (int idx = 0; idx < length; idx++) {
      if (packet[idx] < 16) {
        message += "0"; // pad single hex digits with a 0
      }
      message += String(packet[idx], HEX) + " ";
    }

    const size_t bufferSize = JSON_OBJECT_SIZE(6);
    DynamicJsonDocument root(bufferSize);

    root[packetDirection] = message;

    char buffer[512];
    serializeJson(root, buffer);

    if (!client.publish(titon_debug_topic, buffer)) {
      client.publish(titon_debug_topic, "failed to publish to debug topic");
    }
  }
}

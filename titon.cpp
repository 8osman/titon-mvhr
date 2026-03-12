// Titon MVHR - Complete Control with MAX485 Module
// Board: ESP32 Dev Module (or Lolin32 Lite)
// RS485: MAX485 module with DE/RE control
// Version 2.0 - Updated with Ross Cullen's ESPHome discoveries

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ========== CONFIGURATION ==========
const char* WIFI_SSID = "YourWiFiName";
const char* WIFI_PASSWORD = "YourWiFiPassword";
const char* MQTT_SERVER = "192.168.0.xxx";
const int MQTT_PORT = 1883;
const char* MQTT_USER = "mqtt_user";
const char* MQTT_PASSWORD = "mqtt_pass";
const char* MQTT_CLIENT_ID = "titon_mvhr";

// RS485 Settings with MAX485 Module
const int RS485_RX = 16;      // Connect to RO (Receiver Output) on MAX485
const int RS485_TX = 17;      // Connect to DI (Driver Input) on MAX485
const int RS485_DE = 4;       // Connect to DE (Driver Enable) on MAX485
const int RS485_RE = 4;       // Connect to RE (Receiver Enable) on MAX485 (same pin as DE)
const int RS485_BAUD = 1200;

// Relay Control Pins (connected to 3-channel relay module)
const int RELAY_SW1 = 25;  // SW1: SUMMERboost Disable
const int RELAY_SW2 = 26;  // SW2: Wet Room Boost
const int RELAY_SW3 = 27;  // SW3: Speed 1 Setback / Kitchen Boost

// Humidity Sensor (0-10V via voltage divider)
const int HUMIDITY_PIN = 34;  // GPIO34 (ADC1_CH6)

// MQTT Topics
const char* TOPIC_STATE = "homeassistant/climate/titon_mvhr/state";
const char* TOPIC_COMMAND = "homeassistant/climate/titon_mvhr/command";
const char* TOPIC_AVAILABILITY = "homeassistant/climate/titon_mvhr/availability";
const char* DISCOVERY_PREFIX = "homeassistant";

// ========== STATUS WORD BIT DEFINITIONS ==========
#define STATUS_SUPPLY_FAN_ERROR     0x0001  // Bit 0
#define STATUS_THERMISTOR_ERROR     0x0002  // Bit 1 (general)
#define STATUS_EXTRACT_FAN_ERROR    0x0004  // Bit 2
#define STATUS_EEPROM_ERROR         0x0008  // Bit 3
#define STATUS_SWITCH1_ACTIVE       0x0010  // Bit 4
#define STATUS_SWITCH2_ACTIVE       0x0020  // Bit 5
#define STATUS_SWITCH3_ACTIVE       0x0040  // Bit 6
#define STATUS_LS1_ACTIVE           0x0080  // Bit 7
#define STATUS_LS2_ACTIVE           0x0100  // Bit 8
#define STATUS_ENGINE_ERROR         0x0200  // Bit 9
#define STATUS_SWITCH_ERROR         0x0400  // Bit 10
#define STATUS_ENGINE_RUNNING       0x0800  // Bit 11 (2048 decimal)
#define STATUS_THERM1_ERROR         0x1000  // Bit 12
#define STATUS_THERM2_ERROR         0x2000  // Bit 13
#define STATUS_THERM3_ERROR         0x4000  // Bit 14
#define STATUS_HUMIDITY_ERROR       0x8000  // Bit 15

// ========== GLOBALS ==========
WiFiClient espClient;
PubSubClient mqtt(espClient);

// Sensor Data - Original
float supply_temp = NAN;
float extract_temp = NAN;
float supply_rpm = NAN;
float extract_rpm = NAN;
float current_humidity = NAN;  // External humidity sensor
int current_speed = 2;
bool summer_bypass = false;
bool summerboost_active = false;

// Sensor Data - NEW from Ross Cullen's discoveries
float stale_air_in_temp = NAN;   // Address 030 - Thermistor 1
float stale_air_out_temp = NAN;  // Address 031 - Thermistor 2
float fresh_air_in_temp = NAN;   // Address 032 - Thermistor 3
int internal_humidity = -1;      // Address 036 - Internal RH sensor
int runtime_hours = 0;           // Address 060 - Total runtime
int status_word = 0;             // Address 061 - Status bitmap
int filter_remaining = 0;        // Address 341 - Filter hours left

// Relay States (for Home Assistant feedback)
bool relay_sw1_active = false;
bool relay_sw2_active = false;
bool relay_sw3_active = false;

// Configurable Settings
struct Settings {
  int speed1_supply = 18;
  int speed1_extract = 18;
  int speed2_supply = 40;
  int speed2_extract = 40;
  int speed3_supply = 70;
  int speed3_extract = 70;
  int speed4_supply = 100;
  int speed4_extract = 100;
  int humidity_setpoint = 70;
  int kitchen_overrun = 10;
  int wetroom_overrun = 30;
  int bypass_extract_threshold = 22;
  int bypass_supply_threshold = 15;
  bool summerboost_enabled = true;
} settings;

String rx_buffer = "";
unsigned long last_mqtt_publish = 0;
unsigned long last_heartbeat = 0;
unsigned long last_humidity_read = 0;
unsigned long last_sensor_poll = 0;
int poll_index = 0;
const unsigned long PUBLISH_INTERVAL = 5000;
const unsigned long HUMIDITY_READ_INTERVAL = 5000;
const unsigned long SENSOR_POLL_INTERVAL = 2000;  // Poll one sensor every 2 seconds

// ========== FORWARD DECLARATIONS ==========
void setup_wifi();
void reconnect_mqtt();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void publish_discovery();
void publish_state();
void parse_response(String response);
void decode_status_word(int status);
void set_fan_speed(int speed);
void trigger_boost(int switch_num, unsigned long duration_ms);
void set_relay(int relay_pin, bool state);
float read_humidity();
void rs485_begin_transmit();
void rs485_begin_receive();
void send_rs485_command(const char* cmd);
void poll_mvhr_sensors();

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  Serial.println("\n========================================");
  Serial.println("Titon MVHR - Complete Control System v2.0");
  Serial.println("With MAX485 Module");
  Serial.println("Updated with Ross Cullen's discoveries");
  Serial.println("========================================");
  
  // Initialize RS485 with MAX485 control
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(RS485_DE, OUTPUT);
  pinMode(RS485_RE, OUTPUT);
  rs485_begin_receive();  // Start in receive mode
  Serial.println("RS485 initialized at 1200 baud with MAX485");
  
  // Initialize relay pins
  pinMode(RELAY_SW1, OUTPUT);
  pinMode(RELAY_SW2, OUTPUT);
  pinMode(RELAY_SW3, OUTPUT);
  digitalWrite(RELAY_SW1, LOW);
  digitalWrite(RELAY_SW2, LOW);
  digitalWrite(RELAY_SW3, LOW);
  Serial.println("Relay outputs initialized");
  
  // Initialize humidity sensor ADC
  pinMode(HUMIDITY_PIN, INPUT);
  analogSetAttenuation(ADC_11db);  // 0-3.3V range
  Serial.println("Humidity sensor ADC initialized");
  
  setup_wifi();
  
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqtt_callback);
  mqtt.setBufferSize(2048);  // Increased for larger state messages
  
  delay(1000);
  publish_discovery();
  
  Serial.println("Setup complete!");
  Serial.println("========================================");
}

// ========== MAX485 CONTROL FUNCTIONS ==========
void rs485_begin_transmit() {
  digitalWrite(RS485_DE, HIGH);  // Enable driver
  digitalWrite(RS485_RE, HIGH);  // Disable receiver
  delayMicroseconds(10);         // Small delay for switching
}

void rs485_begin_receive() {
  delayMicroseconds(10);         // Wait for transmission to complete
  digitalWrite(RS485_DE, LOW);   // Disable driver
  digitalWrite(RS485_RE, LOW);   // Enable receiver
}

void send_rs485_command(const char* cmd) {
  rs485_begin_transmit();
  Serial2.print(cmd);
  Serial2.flush();  // Wait for transmission to complete
  rs485_begin_receive();
  Serial.printf("RS485 TX: %s", cmd);
}

// ========== WIFI ==========
void setup_wifi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(" FAILED!");
  }
}

// ========== MQTT RECONNECT ==========
void reconnect_mqtt() {
  if (mqtt.connected()) return;
  
  Serial.print("Connecting to MQTT...");
  
  if (mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD,
                   TOPIC_AVAILABILITY, 0, true, "offline")) {
    Serial.println(" connected!");
    mqtt.publish(TOPIC_AVAILABILITY, "online", true);
    mqtt.subscribe(TOPIC_COMMAND);
  } else {
    Serial.print(" failed, rc=");
    Serial.println(mqtt.state());
  }
}

// ========== MQTT CALLBACK ==========
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("MQTT RX: ");
  Serial.println(message);
  
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, message)) {
    Serial.println("JSON parse failed");
    return;
  }
  
  // Fan speed control (via RS485)
  if (doc.containsKey("fan_speed")) {
    set_fan_speed(doc["fan_speed"]);
  }
  
  // Relay switch control
  if (doc.containsKey("sw1")) {
    bool state = doc["sw1"];
    set_relay(RELAY_SW1, state);
    relay_sw1_active = state;
    Serial.printf("SW1 (SUMMERboost Disable): %s\n", state ? "ON" : "OFF");
  }
  
  if (doc.containsKey("sw2")) {
    bool state = doc["sw2"];
    set_relay(RELAY_SW2, state);
    relay_sw2_active = state;
    Serial.printf("SW2 (Wet Room Boost): %s\n", state ? "ON" : "OFF");
  }
  
  if (doc.containsKey("sw3")) {
    bool state = doc["sw3"];
    set_relay(RELAY_SW3, state);
    relay_sw3_active = state;
    Serial.printf("SW3 (Setback/Kitchen): %s\n", state ? "ON" : "OFF");
  }
  
  // Momentary boost triggers (pulse relay for 2 seconds)
  if (doc.containsKey("trigger_wetroom_boost")) {
    Serial.println("Triggering wet room boost (momentary)");
    trigger_boost(2, 2000);  // SW2 for 2 seconds
  }
  
  if (doc.containsKey("trigger_kitchen_boost")) {
    Serial.println("Triggering kitchen boost (momentary)");
    trigger_boost(3, 2000);  // SW3 for 2 seconds
  }
  
  // NEW: Direct RS485 control commands
  if (doc.containsKey("boost_inhibit")) {
    bool enabled = doc["boost_inhibit"];
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "3260+%05d\r\n", enabled ? 1 : 0);
    send_rs485_command(cmd);
    Serial.printf("Boost Inhibit (Night Mode): %s\n", enabled ? "ENABLED" : "DISABLED");
  }
  
  if (doc.containsKey("summer_bypass_enable")) {
    bool enabled = doc["summer_bypass_enable"];
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "2300+%05d\r\n", enabled ? 1 : 0);
    send_rs485_command(cmd);
    Serial.printf("Summer Bypass: %s\n", enabled ? "ENABLED" : "DISABLED");
  }
  
  // CRITICAL FIX: SUMMERboost uses INVERTED logic!
  if (doc.containsKey("summerboost_enable")) {
    bool enabled = doc["summerboost_enable"];
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "2900+%05d\r\n", enabled ? 0 : 1);  // INVERTED!
    send_rs485_command(cmd);
    Serial.printf("SUMMERboost: %s (wrote %d - inverted logic)\n", 
                  enabled ? "ENABLED" : "DISABLED", 
                  enabled ? 0 : 1);
  }
  
  // Factory reset with safety confirmation
  if (doc.containsKey("factory_reset")) {
    bool confirm = doc["factory_reset"];
    if (confirm) {
      Serial.println("⚠️⚠️⚠️  FACTORY RESET REQUESTED!");
      Serial.println("Sending reset command in 5 seconds...");
      delay(5000);
      send_rs485_command("0680+21930\r\n");
      Serial.println("Factory reset command sent!");
    }
  }
  
  // Settings updates (stored in memory)
  if (doc.containsKey("speed1_supply")) settings.speed1_supply = doc["speed1_supply"];
  if (doc.containsKey("speed1_extract")) settings.speed1_extract = doc["speed1_extract"];
  if (doc.containsKey("speed2_supply")) settings.speed2_supply = doc["speed2_supply"];
  if (doc.containsKey("speed2_extract")) settings.speed2_extract = doc["speed2_extract"];
  if (doc.containsKey("speed3_supply")) settings.speed3_supply = doc["speed3_supply"];
  if (doc.containsKey("speed3_extract")) settings.speed3_extract = doc["speed3_extract"];
  if (doc.containsKey("speed4_supply")) settings.speed4_supply = doc["speed4_supply"];
  if (doc.containsKey("speed4_extract")) settings.speed4_extract = doc["speed4_extract"];
  if (doc.containsKey("humidity_setpoint")) settings.humidity_setpoint = doc["humidity_setpoint"];
  if (doc.containsKey("kitchen_overrun")) settings.kitchen_overrun = doc["kitchen_overrun"];
  if (doc.containsKey("wetroom_overrun")) settings.wetroom_overrun = doc["wetroom_overrun"];
  if (doc.containsKey("bypass_extract_threshold")) settings.bypass_extract_threshold = doc["bypass_extract_threshold"];
  if (doc.containsKey("bypass_supply_threshold")) settings.bypass_supply_threshold = doc["bypass_supply_threshold"];
  if (doc.containsKey("summerboost_enabled")) settings.summerboost_enabled = doc["summerboost_enabled"];
}

// ========== HOME ASSISTANT DISCOVERY ==========
void publish_discovery() {
  Serial.println("Publishing Home Assistant discovery...");
  
  // Climate entity
  {
    char topic[128];
    snprintf(topic, sizeof(topic), "%s/climate/titon_mvhr/config", DISCOVERY_PREFIX);
    
    StaticJsonDocument<768> doc;
    doc["name"] = "Titon MVHR";
    doc["unique_id"] = "titon_mvhr_climate";
    doc["mode_command_topic"] = TOPIC_COMMAND;
    doc["mode_state_topic"] = TOPIC_STATE;
    doc["mode_state_template"] = "{{ value_json.mode }}";
    doc["modes"][0] = "off";
    doc["modes"][1] = "fan_only";
    
    doc["fan_mode_command_topic"] = TOPIC_COMMAND;
    doc["fan_mode_state_topic"] = TOPIC_STATE;
    doc["fan_mode_state_template"] = "{{ value_json.fan_mode }}";
    doc["fan_modes"][0] = "low";
    doc["fan_modes"][1] = "medium";
    doc["fan_modes"][2] = "high";
    doc["fan_modes"][3] = "auto";
    
    doc["current_temperature_topic"] = TOPIC_STATE;
    doc["current_temperature_template"] = "{{ value_json.supply_temp }}";
    doc["temperature_unit"] = "C";
    doc["availability_topic"] = TOPIC_AVAILABILITY;
    
    JsonObject dev = doc.createNestedObject("device");
    dev["identifiers"][0] = "titon_mvhr";
    dev["name"] = "Titon MVHR";
    dev["model"] = "HRV1.6 Q Plus HMB";
    dev["manufacturer"] = "Titon";
    dev["sw_version"] = "v2.0";
    
    char buffer[768];
    serializeJson(doc, buffer);
    mqtt.publish(topic, buffer, true);
    delay(100);
  }
  
  // Sensors macro
  #define PUBLISH_SENSOR(id, name, unit, dev_class) { \
    char topic[128]; \
    snprintf(topic, sizeof(topic), "%s/sensor/titon_mvhr/%s/config", DISCOVERY_PREFIX, id); \
    StaticJsonDocument<384> doc; \
    doc["name"] = name; \
    doc["unique_id"] = String("titon_mvhr_") + id; \
    doc["state_topic"] = TOPIC_STATE; \
    doc["value_template"] = String("{{ value_json.") + id + " }}"; \
    doc["availability_topic"] = TOPIC_AVAILABILITY; \
    if (strlen(unit) > 0) doc["unit_of_measurement"] = unit; \
    if (strlen(dev_class) > 0) doc["device_class"] = dev_class; \
    JsonObject dev = doc.createNestedObject("device"); \
    dev["identifiers"][0] = "titon_mvhr"; \
    char buffer[384]; \
    serializeJson(doc, buffer); \
    mqtt.publish(topic, buffer, true); \
    delay(50); \
  }
  
  // Original sensors
  PUBLISH_SENSOR("supply_temp", "Supply Temperature", "°C", "temperature");
  PUBLISH_SENSOR("extract_temp", "Extract Temperature", "°C", "temperature");
  PUBLISH_SENSOR("supply_rpm", "Supply Fan RPM", "RPM", "");
  PUBLISH_SENSOR("extract_rpm", "Extract Fan RPM", "RPM", "");
  PUBLISH_SENSOR("current_speed", "Current Speed", "", "");
  PUBLISH_SENSOR("humidity", "External Humidity", "%", "humidity");
  
  // NEW: Three thermistors from Ross's discoveries
  PUBLISH_SENSOR("stale_air_in_temp", "Stale Air In Temperature", "°C", "temperature");
  PUBLISH_SENSOR("stale_air_out_temp", "Stale Air Out Temperature", "°C", "temperature");
  PUBLISH_SENSOR("fresh_air_in_temp", "Fresh Air In Temperature", "°C", "temperature");
  
  // NEW: Additional sensors
  PUBLISH_SENSOR("internal_humidity", "Internal Humidity", "%", "humidity");
  PUBLISH_SENSOR("runtime_hours", "Runtime Hours", "h", "duration");
  PUBLISH_SENSOR("filter_remaining", "Filter Remaining", "h", "duration");
  PUBLISH_SENSOR("status_word", "Status Word (Raw)", "", "");
  
  // Binary sensors macro
  #define PUBLISH_BINARY(id, name, dev_class) { \
    char topic[128]; \
    snprintf(topic, sizeof(topic), "%s/binary_sensor/titon_mvhr/%s/config", DISCOVERY_PREFIX, id); \
    StaticJsonDocument<384> doc; \
    doc["name"] = name; \
    doc["unique_id"] = String("titon_mvhr_") + id; \
    doc["state_topic"] = TOPIC_STATE; \
    doc["value_template"] = String("{{ value_json.") + id + " }}"; \
    doc["payload_on"] = "true"; \
    doc["payload_off"] = "false"; \
    doc["availability_topic"] = TOPIC_AVAILABILITY; \
    if (strlen(dev_class) > 0) doc["device_class"] = dev_class; \
    JsonObject dev = doc.createNestedObject("device"); \
    dev["identifiers"][0] = "titon_mvhr"; \
    char buffer[384]; \
    serializeJson(doc, buffer); \
    mqtt.publish(topic, buffer, true); \
    delay(50); \
  }
  
  // Original binary sensors
  PUBLISH_BINARY("summer_bypass", "Summer Bypass Active", "");
  PUBLISH_BINARY("summerboost", "SUMMERboost Active", "");
  
  // NEW: Status word decoded sensors
  PUBLISH_BINARY("engine_running", "Engine Running", "running");
  PUBLISH_BINARY("supply_fan_error", "Supply Fan Error", "problem");
  PUBLISH_BINARY("extract_fan_error", "Extract Fan Error", "problem");
  PUBLISH_BINARY("thermistor_error", "Thermistor Error (General)", "problem");
  PUBLISH_BINARY("therm1_error", "Thermistor 1 Error", "problem");
  PUBLISH_BINARY("therm2_error", "Thermistor 2 Error", "problem");
  PUBLISH_BINARY("therm3_error", "Thermistor 3 Error", "problem");
  PUBLISH_BINARY("humidity_sensor_error", "Humidity Sensor Error", "problem");
  PUBLISH_BINARY("eeprom_error", "EEPROM Error", "problem");
  PUBLISH_BINARY("engine_error", "Engine Error", "problem");
  PUBLISH_BINARY("switch_error", "Switch Error", "problem");
  
  // Switch entities
  #define PUBLISH_SWITCH(id, name) { \
    char topic[128]; \
    snprintf(topic, sizeof(topic), "%s/switch/titon_mvhr/%s/config", DISCOVERY_PREFIX, id); \
    StaticJsonDocument<384> doc; \
    doc["name"] = name; \
    doc["unique_id"] = String("titon_mvhr_") + id; \
    doc["state_topic"] = TOPIC_STATE; \
    doc["command_topic"] = TOPIC_COMMAND; \
    doc["value_template"] = String("{{ value_json.") + id + " }}"; \
    doc["payload_on"] = String("{\"") + id + "\": true}"; \
    doc["payload_off"] = String("{\"") + id + "\": false}"; \
    doc["state_on"] = "true"; \
    doc["state_off"] = "false"; \
    doc["availability_topic"] = TOPIC_AVAILABILITY; \
    JsonObject dev = doc.createNestedObject("device"); \
    dev["identifiers"][0] = "titon_mvhr"; \
    char buffer[384]; \
    serializeJson(doc, buffer); \
    mqtt.publish(topic, buffer, true); \
    delay(50); \
  }
  
  PUBLISH_SWITCH("sw1", "SUMMERboost Disable (SW1)");
  PUBLISH_SWITCH("sw2", "Wet Room Boost (SW2)");
  PUBLISH_SWITCH("sw3", "Setback/Kitchen (SW3)");
  
  // NEW: Direct RS485 control switches
  PUBLISH_SWITCH("boost_inhibit", "Boost Inhibit (Night Mode)");
  PUBLISH_SWITCH("summer_bypass_enable", "Summer Bypass Enable");
  PUBLISH_SWITCH("summerboost_enable", "SUMMERboost Enable");
  
  // Button entities
  #define PUBLISH_BUTTON(id, name, cmd_key) { \
    char topic[128]; \
    snprintf(topic, sizeof(topic), "%s/button/titon_mvhr/%s/config", DISCOVERY_PREFIX, id); \
    StaticJsonDocument<384> doc; \
    doc["name"] = name; \
    doc["unique_id"] = String("titon_mvhr_") + id; \
    doc["command_topic"] = TOPIC_COMMAND; \
    doc["payload_press"] = String("{\"") + cmd_key + "\": true}"; \
    doc["availability_topic"] = TOPIC_AVAILABILITY; \
    JsonObject dev = doc.createNestedObject("device"); \
    dev["identifiers"][0] = "titon_mvhr"; \
    char buffer[384]; \
    serializeJson(doc, buffer); \
    mqtt.publish(topic, buffer, true); \
    delay(50); \
  }
  
  PUBLISH_BUTTON("trigger_wetroom", "Trigger Wet Room Boost", "trigger_wetroom_boost");
  PUBLISH_BUTTON("trigger_kitchen", "Trigger Kitchen Boost", "trigger_kitchen_boost");
  PUBLISH_BUTTON("factory_reset_btn", "Factory Reset MVHR", "factory_reset");
  
  // Number entities
  #define PUBLISH_NUMBER(id, name, min_v, max_v) { \
    char topic[128]; \
    snprintf(topic, sizeof(topic), "%s/number/titon_mvhr/%s/config", DISCOVERY_PREFIX, id); \
    StaticJsonDocument<384> doc; \
    doc["name"] = name; \
    doc["unique_id"] = String("titon_mvhr_") + id; \
    doc["state_topic"] = TOPIC_STATE; \
    doc["command_topic"] = TOPIC_COMMAND; \
    doc["value_template"] = String("{{ value_json.") + id + " }}"; \
    doc["command_template"] = String("{\"") + id + "\": {{ value }}}"; \
    doc["min"] = min_v; \
    doc["max"] = max_v; \
    doc["step"] = 1; \
    doc["mode"] = "slider"; \
    doc["availability_topic"] = TOPIC_AVAILABILITY; \
    JsonObject dev = doc.createNestedObject("device"); \
    dev["identifiers"][0] = "titon_mvhr"; \
    char buffer[384]; \
    serializeJson(doc, buffer); \
    mqtt.publish(topic, buffer, true); \
    delay(50); \
  }
  
  PUBLISH_NUMBER("speed1_supply", "Speed 1 Supply %", 14, 100);
  PUBLISH_NUMBER("speed1_extract", "Speed 1 Extract %", 14, 100);
  PUBLISH_NUMBER("speed2_supply", "Speed 2 Supply %", 14, 100);
  PUBLISH_NUMBER("speed2_extract", "Speed 2 Extract %", 14, 100);
  PUBLISH_NUMBER("speed3_supply", "Speed 3 Supply %", 14, 100);
  PUBLISH_NUMBER("speed3_extract", "Speed 3 Extract %", 14, 100);
  PUBLISH_NUMBER("speed4_supply", "Speed 4 Supply %", 14, 100);
  PUBLISH_NUMBER("speed4_extract", "Speed 4 Extract %", 14, 100);
  PUBLISH_NUMBER("humidity_setpoint", "Humidity Setpoint", 30, 100);
  PUBLISH_NUMBER("kitchen_overrun", "Kitchen Timer (min)", 0, 60);
  PUBLISH_NUMBER("wetroom_overrun", "Wet Room Timer (min)", 0, 60);
  PUBLISH_NUMBER("bypass_extract_threshold", "Bypass Extract °C", 17, 35);
  PUBLISH_NUMBER("bypass_supply_threshold", "Bypass Supply °C", 10, 20);
  
  Serial.println("Discovery complete!");
}

// ========== PUBLISH STATE ==========
void publish_state() {
  if (!mqtt.connected()) return;
  
  StaticJsonDocument<2048> doc;  // Increased for all new sensors
  
  // Original temperature sensors
  doc["supply_temp"] = supply_temp;
  doc["extract_temp"] = extract_temp;
  
  // NEW: Three thermistors
  doc["stale_air_in_temp"] = stale_air_in_temp;
  doc["stale_air_out_temp"] = stale_air_out_temp;
  doc["fresh_air_in_temp"] = fresh_air_in_temp;
  
  // RPM and speed
  doc["supply_rpm"] = supply_rpm;
  doc["extract_rpm"] = extract_rpm;
  doc["current_speed"] = current_speed;
  
  // Humidity sensors
  doc["humidity"] = current_humidity;          // External sensor
  doc["internal_humidity"] = internal_humidity; // NEW: Internal sensor
  
  // NEW: Runtime and filter
  doc["runtime_hours"] = runtime_hours;
  doc["filter_remaining"] = filter_remaining;
  doc["status_word"] = status_word;
  
  // Status flags (decoded from status word)
  doc["engine_running"] = (status_word & STATUS_ENGINE_RUNNING) != 0;
  doc["supply_fan_error"] = (status_word & STATUS_SUPPLY_FAN_ERROR) != 0;
  doc["extract_fan_error"] = (status_word & STATUS_EXTRACT_FAN_ERROR) != 0;
  doc["thermistor_error"] = (status_word & STATUS_THERMISTOR_ERROR) != 0;
  doc["therm1_error"] = (status_word & STATUS_THERM1_ERROR) != 0;
  doc["therm2_error"] = (status_word & STATUS_THERM2_ERROR) != 0;
  doc["therm3_error"] = (status_word & STATUS_THERM3_ERROR) != 0;
  doc["humidity_sensor_error"] = (status_word & STATUS_HUMIDITY_ERROR) != 0;
  doc["eeprom_error"] = (status_word & STATUS_EEPROM_ERROR) != 0;
  doc["engine_error"] = (status_word & STATUS_ENGINE_ERROR) != 0;
  doc["switch_error"] = (status_word & STATUS_SWITCH_ERROR) != 0;
  
  // System state
  doc["summer_bypass"] = summer_bypass;
  doc["summerboost"] = summerboost_active;
  
  // Relay states
  doc["sw1"] = relay_sw1_active;
  doc["sw2"] = relay_sw2_active;
  doc["sw3"] = relay_sw3_active;
  
  // Climate entity
  doc["mode"] = (current_speed > 0) ? "fan_only" : "off";
  String fan_mode = "medium";
  if (current_speed == 1) fan_mode = "low";
  else if (current_speed == 3) fan_mode = "high";
  else if (current_speed == 4) fan_mode = "auto";
  doc["fan_mode"] = fan_mode;
  
  // Settings
  doc["speed1_supply"] = settings.speed1_supply;
  doc["speed1_extract"] = settings.speed1_extract;
  doc["speed2_supply"] = settings.speed2_supply;
  doc["speed2_extract"] = settings.speed2_extract;
  doc["speed3_supply"] = settings.speed3_supply;
  doc["speed3_extract"] = settings.speed3_extract;
  doc["speed4_supply"] = settings.speed4_supply;
  doc["speed4_extract"] = settings.speed4_extract;
  doc["humidity_setpoint"] = settings.humidity_setpoint;
  doc["kitchen_overrun"] = settings.kitchen_overrun;
  doc["wetroom_overrun"] = settings.wetroom_overrun;
  doc["bypass_extract_threshold"] = settings.bypass_extract_threshold;
  doc["bypass_supply_threshold"] = settings.bypass_supply_threshold;
  doc["summerboost_enabled"] = settings.summerboost_enabled;
  
  char buffer[2048];
  serializeJson(doc, buffer);
  mqtt.publish(TOPIC_STATE, buffer);
}

// ========== RS485 PARSING ==========
void parse_response(String response) {
  int sign_pos = -1;
  for (unsigned int i = 0; i < response.length(); i++) {
    if (response[i] == '+' || response[i] == '-') {
      sign_pos = i;
      break;
    }
  }
  
  if (sign_pos == -1) return;
  
  int address = response.substring(0, sign_pos).toInt();
  int value = response.substring(sign_pos).toInt();
  
  // Check for error response (faulty sensor returns -99999)
  if (value == -99999) {
    Serial.printf("⚠️  Address %d returned error (-99999) - FAULTY SENSOR!\n", address);
    return;  // Don't update the value
  }
  
  switch (address) {
    // NEW: Three thermistors (0.1°C resolution)
    case 30:
      stale_air_in_temp = value / 10.0;
      Serial.printf("Stale In Temp: %.1f°C\n", stale_air_in_temp);
      break;
    case 31:
      stale_air_out_temp = value / 10.0;
      Serial.printf("Stale Out Temp: %.1f°C\n", stale_air_out_temp);
      break;
    case 32:
      fresh_air_in_temp = value / 10.0;
      Serial.printf("Fresh In Temp: %.1f°C\n", fresh_air_in_temp);
      break;
      
    // NEW: Internal humidity (1% resolution)
    case 36:
      internal_humidity = value;
      Serial.printf("Internal Humidity: %d%%\n", internal_humidity);
      break;
      
    // NEW: Runtime hours
    case 60:
      runtime_hours = value;
      Serial.printf("Runtime Hours: %d\n", runtime_hours);
      break;
      
    // NEW: Status word (decode bits)
    case 61:
      status_word = value;
      decode_status_word(value);
      break;
      
    // NEW: Filter remaining
    case 341:
      filter_remaining = value;
      Serial.printf("Filter Remaining: %d hours\n", filter_remaining);
      break;
      
    // Original sensors
    case 380:
      supply_rpm = value;
      break;
    case 381:
      extract_rpm = value;
      break;
    case 382:
      supply_temp = value / 10.0;
      break;
    case 383:
      extract_temp = value / 10.0;
      break;
    case 384:
      current_speed = value;
      break;
    case 385:
      summer_bypass = (value & 0x01) != 0;
      summerboost_active = (value & 0x02) != 0;
      break;
  }
}

// ========== NEW: STATUS WORD DECODER ==========
void decode_status_word(int status) {
  Serial.println("=== STATUS WORD DECODING ===");
  Serial.printf("Raw value: %d (0x%04X)\n", status, status);
  
  if (status & STATUS_SUPPLY_FAN_ERROR)    Serial.println("⚠️  Supply Fan Error");
  if (status & STATUS_THERMISTOR_ERROR)    Serial.println("⚠️  Thermistor Error (General)");
  if (status & STATUS_EXTRACT_FAN_ERROR)   Serial.println("⚠️  Extract Fan Error");
  if (status & STATUS_EEPROM_ERROR)        Serial.println("⚠️  EEPROM Error");
  if (status & STATUS_SWITCH1_ACTIVE)      Serial.println("ℹ️  Switch 1 Active");
  if (status & STATUS_SWITCH2_ACTIVE)      Serial.println("ℹ️  Switch 2 Active");
  if (status & STATUS_SWITCH3_ACTIVE)      Serial.println("ℹ️  Switch 3 Active");
  if (status & STATUS_LS1_ACTIVE)          Serial.println("ℹ️  LS1 Active");
  if (status & STATUS_LS2_ACTIVE)          Serial.println("ℹ️  LS2 Active");
  if (status & STATUS_ENGINE_ERROR)        Serial.println("⚠️  Engine Error");
  if (status & STATUS_SWITCH_ERROR)        Serial.println("⚠️  Switch Error");
  if (status & STATUS_ENGINE_RUNNING)      Serial.println("✅ Engine Running");
  if (status & STATUS_THERM1_ERROR)        Serial.println("⚠️  Thermistor 1 Error");
  if (status & STATUS_THERM2_ERROR)        Serial.println("⚠️  Thermistor 2 Error");
  if (status & STATUS_THERM3_ERROR)        Serial.println("⚠️  Thermistor 3 Error");
  if (status & STATUS_HUMIDITY_ERROR)      Serial.println("⚠️  Humidity Sensor Error");
  
  Serial.println("===========================");
}

// ========== NEW: ROTATING SENSOR POLL ==========
void poll_mvhr_sensors() {
  if (millis() - last_sensor_poll < SENSOR_POLL_INTERVAL) return;
  last_sensor_poll = millis();
  
  // Rotate through sensors to avoid bus saturation
  // Poll one sensor every 2 seconds = all 10 sensors every 20 seconds
  switch (poll_index) {
    case 0: send_rs485_command("0301+00000\r\n"); break;  // Stale In
    case 1: send_rs485_command("0311+00000\r\n"); break;  // Stale Out
    case 2: send_rs485_command("0321+00000\r\n"); break;  // Fresh In
    case 3: send_rs485_command("0361+00000\r\n"); break;  // Internal Humidity
    case 4: send_rs485_command("0601+00000\r\n"); break;  // Runtime Hours
    case 5: send_rs485_command("0611+00000\r\n"); break;  // Status Word
    case 6: send_rs485_command("3411+00000\r\n"); break;  // Filter Remaining
    case 7: send_rs485_command("3821+00000\r\n"); break;  // Supply Temp
    case 8: send_rs485_command("3831+00000\r\n"); break;  // Extract Temp
    case 9: send_rs485_command("3841+00000\r\n"); break;  // Current Speed
  }
  
  poll_index = (poll_index + 1) % 10;
}

// ========== FAN SPEED CONTROL (RS485) ==========
void set_fan_speed(int speed) {
  if (speed < 1 || speed > 4) return;
  
  int speed_value = 0;
  switch (speed) {
    case 1: speed_value = 1; break;
    case 2: speed_value = 2; break;
    case 3: speed_value = 4; break;
    case 4: speed_value = 8; break;
  }
  
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "3840+%05d\r\n", speed_value);
  send_rs485_command(cmd);
  Serial.printf("Set speed to %d (value=%d)\n", speed, speed_value);
}

// ========== RELAY CONTROL ==========
void set_relay(int relay_pin, bool state) {
  digitalWrite(relay_pin, state ? HIGH : LOW);
  Serial.printf("Relay on pin %d: %s\n", relay_pin, state ? "ON" : "OFF");
}

void trigger_boost(int switch_num, unsigned long duration_ms) {
  int relay_pin;
  switch (switch_num) {
    case 1: relay_pin = RELAY_SW1; break;
    case 2: relay_pin = RELAY_SW2; break;
    case 3: relay_pin = RELAY_SW3; break;
    default: return;
  }
  
  Serial.printf("Pulsing SW%d relay for %lu ms\n", switch_num, duration_ms);
  digitalWrite(relay_pin, HIGH);
  delay(duration_ms);
  digitalWrite(relay_pin, LOW);
  Serial.printf("SW%d pulse complete - PCB will handle overrun timer\n", switch_num);
}

// ========== HUMIDITY SENSOR ==========
float read_humidity() {
  // Read ADC multiple times and average for stability
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += analogRead(HUMIDITY_PIN);
    delay(10);
  }
  int raw = sum / 5;
  
  // Convert to voltage (0-3.3V)
  float voltage = (raw / 4095.0) * 3.3;
  
  // Convert to sensor voltage (0-10V)
  // Voltage divider: R1=68k, R2=22k
  float sensor_voltage = voltage * ((68.0 + 22.0) / 22.0);
  
  // Convert to humidity percentage (0-100%)
  float humidity = sensor_voltage * 10.0;
  
  // Clamp to valid range
  if (humidity < 0) humidity = 0;
  if (humidity > 100) humidity = 100;
  
  return humidity;
}

// ========== MAIN LOOP ==========
void loop() {
  // WiFi check
  if (WiFi.status() != WL_CONNECTED) {
    setup_wifi();
  }
  
  // MQTT check
  if (!mqtt.connected()) {
    reconnect_mqtt();
  }
  mqtt.loop();
  
  // Heartbeat
  if (millis() - last_heartbeat > 5000) {
    Serial.printf("Status - WiFi:%s MQTT:%s ExtRH:%.1f%% IntRH:%d%% Runtime:%dh Filter:%dh\n",
                  WiFi.status() == WL_CONNECTED ? "OK" : "X",
                  mqtt.connected() ? "OK" : "X",
                  current_humidity,
                  internal_humidity,
                  runtime_hours,
                  filter_remaining);
    last_heartbeat = millis();
  }
  
  // NEW: Rotating sensor poll (one sensor every 2 seconds)
  poll_mvhr_sensors();
  
  // Read RS485 (always in receive mode unless transmitting)
  while (Serial2.available()) {
    char c = Serial2.read();
    
    if (c == '\n' || c == '\r') {
      if (rx_buffer.length() > 0) {
        Serial.printf("RS485 RX: %s\n", rx_buffer.c_str());
        parse_response(rx_buffer);
        rx_buffer = "";
      }
    } else if (c >= 32 && c <= 126) {
      rx_buffer += c;
      if (rx_buffer.length() > 100) rx_buffer = "";
    }
  }
  
  // Read external humidity sensor
  if (millis() - last_humidity_read > HUMIDITY_READ_INTERVAL) {
    current_humidity = read_humidity();
    last_humidity_read = millis();
  }
  
  // Publish state
  if (millis() - last_mqtt_publish > PUBLISH_INTERVAL) {
    publish_state();
    last_mqtt_publish = millis();
  }
}

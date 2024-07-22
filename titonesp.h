const char* ota_password = "***";

// wifi settings
const char* ssid     = "***";
const char* password = "***";

// mqtt server settings
const char* mqtt_server   = "192.168.1.*";
const int mqtt_port       = 1883;
const char* mqtt_username = "***";
const char* mqtt_password = "***";

// mqtt client settings
const char* client_id                   = "titon"; // Must be unique on the MQTT network

const char* vallox_temp_topic         = "titon/temp"; // temperature topic
const char* vallox_set_topic          = "titon/set"; // set topic
const char* vallox_state_topic        = "titon/state"; // dynamically updatig values (on/off, temp, etc ...)
const char* vallox_debug_topic        = "titon/debug"; // debug topic

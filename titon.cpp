// =======================================
// Titon DIGIT SE COMMUNICATION PROTOCOL
// =======================================

#include "Titon.h"

// tn fan speed (1-8) conversion table
const uint8_t tnFanSpeeds[] = {
  tn_FAN_SPEED_1,
  tn_FAN_SPEED_2,
  tn_FAN_SPEED_3,
  tn_FAN_SPEED_4,
  tn_FAN_SPEED_5,
  tn_FAN_SPEED_6,
  tn_FAN_SPEED_7,
  tn_FAN_SPEED_8
};

// tn NTC temperature conversion table
const int8_t tnTemps[] = {
  -74, -70, -66, -62, -59, -56, -54, -52, -50, -48, // 0x00 - 0x09
  -47, -46, -44, -43, -42, -41, -40, -39, -38, -37, // 0x0a - 0x13
  -36, -35, -34, -33, -33, -32, -31, -30, -30, -29, // 0x14 - 0x1d
  -28, -28, -27, -27, -26, -25, -25, -24, -24, -23, // 0x1e - 0x27
  -23, -22, -22, -21, -21, -20, -20, -19, -19, -19, // 0x28 - 0x31
  -18, -18, -17, -17, -16, -16, -16, -15, -15, -14, // 0x32 - 0x3b
  -14, -14, -13, -13, -12, -12, -12, -11, -11, -11, // 0x3c - 0x45
  -10, -10, -9, -9, -9, -8, -8, -8, -7, -7,         // 0x46 - 0x4f
  -7, -6, -6, -6, -5, -5, -5, -4, -4, -4,           // 0x50 - 0x59
  -3, -3, -3, -2, -2, -2, -1, -1, -1, -1,           // 0x5a - 0x63
  0,  0,  0,  1,  1,  1,  2,  2,  2,  3,            // 0x64 - 0x6d
  3,  3,  4,  4,  4,  5,  5,  5,  5,  6,            // 0x6e - 0x77
  6,  6,  7,  7,  7,  8,  8,  8,  9,  9,            // 0x78 - 0x81
  9, 10, 10, 10, 11, 11, 11, 12, 12, 12,            // 0x82 - 0x8b
  13, 13, 13, 14, 14, 14, 15, 15, 15, 16,           // 0x8c - 0x95
  16, 16, 17, 17, 18, 18, 18, 19, 19, 19,           // 0x96 - 0x9f
  20, 20, 21, 21, 21, 22, 22, 22, 23, 23,           // 0xa0 - 0xa9
  24, 24, 24, 25, 25, 26, 26, 27, 27, 27,           // 0xaa - 0xb3
  28, 28, 29, 29, 30, 30, 31, 31, 32, 32,           // 0xb4 - 0xbd
  33, 33, 34, 34, 35, 35, 36, 36, 37, 37,           // 0xbe - 0xc7
  38, 38, 39, 40, 40, 41, 41, 42, 43, 43,           // 0xc8 - 0xd1
  44, 45, 45, 46, 47, 48, 48, 49, 50, 51,           // 0xd2 - 0xdb
  52, 53, 53, 54, 55, 56, 57, 59, 60, 61,           // 0xdc - 0xe5
  62, 63, 65, 66, 68, 69, 71, 73, 75, 77,           // 0xe6 - 0xef
  79, 81, 82, 86, 90, 93, 97, 100, 100, 100,        // 0xf0 - 0xf9
  100, 100, 100, 100, 100, 100                      // 0xfa - 0xff
};

// public

Titon::Titon() {
  Titon(false);
}

Titon::Titon(boolean debug) {
  isDebug = debug;
}

bool Titon::connect(HardwareSerial *s) {
  serial = s;
  serial->begin(9600, SERIAL_8N1);

  fullInitDone = false;

  requestConfig();

  return true;
}

void Titon::requestConfig() {
  sendStatusReq();
  sendIO08Req();
  sendFanSpeedReq();
  sendDefaultFanSpeedReq();
  sendRhReq();
  sendServicePeriodReq();
  sendServiceCounterReq();
  sendHeatingTargetReq();

  sendFlags06Req();
  sendProgramReq();

  // Temperature values are not needed to request, they are updated automatically
  // RH values are not needed to request, they are updated automatically

  // Set request time for all configurations
  unsigned long now = millis();
  data.updated = millis();
  lastRequested = now;
}

void Titon::loop() {
  byte message[tn_MSG_LENGTH];

  // read and decode as long as messages are available
  while (readMessage(message)) {
    // Inform with callback about message
    decodeMessage(message);
  }

  // query for data that can change without notice
  unsigned long now = millis();
  if (now - lastRequested > QUERY_INTERVAL) {
    lastRequested = now;

    if (isStatusInitDone()) {
      sendIO08Req();
      sendServiceCounterReq();
    }
  }

  if (now - lastRetryLoop > RETRY_INTERVAL) {
    retryLoop();
  }
}

// setters
// these will set data both in the bus and cache
void Titon::setFanSpeed(int speed) {
  if (speed <= tn_MAX_FAN_SPEED) {
    setVariable(tn_VARIABLE_FAN_SPEED, fanSpeed2Hex(speed));
    data.fan_speed.value = speed;
    statusChangedCallback();
  }
}

void Titon::setDefaultFanSpeed(int speed) {
  if (speed < tn_MAX_FAN_SPEED) {
    setVariable(tn_VARIABLE_DEFAULT_FAN_SPEED, fanSpeed2Hex(speed));
    data.default_fan_speed.value = speed;
    statusChangedCallback();
  }
}

// Status variables
void Titon::setOn() {

  if (setStatusVariable(tn_VARIABLE_STATUS, data.status.value | tn_STATUS_FLAG_POWER)) {
    data.is_on.value = true;
    statusChangedCallback();
  }
}

void Titon::setOff() {
  if (setStatusVariable(tn_VARIABLE_STATUS, data.status.value & ~tn_STATUS_FLAG_POWER)) {
    data.is_on.value = false;
    statusChangedCallback();
  }
}

void Titon::setRhModeOn() {
  if (setStatusVariable(tn_VARIABLE_STATUS, data.status.value | tn_STATUS_FLAG_RH)) {
    data.is_rh_mode.value = true;
    statusChangedCallback();
  }
}

void Titon::setRhModeOff() {
  if (setStatusVariable(tn_VARIABLE_STATUS, data.status.value & ~tn_STATUS_FLAG_RH)) {
    data.is_rh_mode.value = false;
    statusChangedCallback();
  }
}

void Titon::setHeatingModeOn() {
  // Don't set if already active. Titon seems to reset to default speed if same mode is set twice
  if (data.status.value & tn_STATUS_FLAG_HEATING_MODE) {
    debugPrintCallback("Heating mode is already on!");
    statusChangedCallback();
  }
  else if (setStatusVariable(tn_VARIABLE_STATUS, data.status.value | tn_STATUS_FLAG_HEATING_MODE)) {
    data.is_heating_mode.value = true;
    statusChangedCallback();
  }
}

void Titon::setHeatingModeOff() {
  // Don't set if already active. Titon seems to reset to default speed if same mode is set twice
  if (!(data.status.value & tn_STATUS_FLAG_HEATING_MODE)) {
    debugPrintCallback("Heating mode is already off!");
    statusChangedCallback();
  }
  else if (setStatusVariable(tn_VARIABLE_STATUS, data.status.value & ~tn_STATUS_FLAG_HEATING_MODE)) {
    data.is_heating_mode.value = false;
    statusChangedCallback();
  }
}

boolean Titon::setStatusVariable(byte variable, byte value) {
  if (!statusMutex) {
    statusMutex = true; // lock sending status again
    // Status is only allowed to send to specific mainboard
    setVariable(variable, value, tn_MSG_MAINBOARD_1);

    // Clear the retry loop to prevent retry loops to break in before getting reply
    lastRetryLoop = millis();
    return true;
  }

  return false;
}

void Titon::setServicePeriod(int months) {
  if (months >= 0 && months < 256) {
    setVariable(tn_VARIABLE_SERVICE_PERIOD, months);
    data.service_period.value = months;
    statusChangedCallback();
  }
}

void Titon::setServiceCounter(int months) {
  if (months >= 0 && months < 256) {
    setVariable(tn_VARIABLE_SERVICE_COUNTER, months);
    data.service_counter.value = months;
    statusChangedCallback();
  }
}

void Titon::setHeatingTarget(int cel) {
  if (cel >= 10 && cel <= 27) {
    byte hex = cel2Ntc(cel);
    setVariable(tn_VARIABLE_HEATING_TARGET, hex);
    data.heating_target.value = cel;
    statusChangedCallback();
  }
}

void Titon::setSwitchOn() {
  // Activate boost/fireplace
  setVariable(tn_VARIABLE_FLAGS_06, data.flags06.value | tn_06_FIREPLACE_FLAG_ACTIVATE);
}

void Titon::setDebug(bool debug) {
  isDebug = debug;
  statusChangedCallback();
}

boolean Titon::isInitOk() {
  return fullInitDone;
}

//Callback setters
void Titon::setPacketCallback(PACKET_CALLBACK_SIGNATURE) {
  this->packetCallback = packetCallback;
}

void Titon::setStatusChangedCallback(STATUS_CHANGED_CALLBACK_SIGNATURE) {
  this->statusChangedCallback = statusChangedCallback;
}

void Titon::setDebugPrintCallback(DEBUG_PRINT_CALLBACK_SIGNATURE) {
  this->debugPrintCallback = debugPrintCallback;
}

void Titon::setTemperatureChangedCallback(TEMPERATURE_CHANGED_CALLBACK_SIGNATURE) {
  this->temperatureChangedCallback = temperatureChangedCallback;
}

// Getters
unsigned long Titon::getUpdated() {
  return data.updated;
}

int Titon::getInsideTemp() {
  return data.t_inside.value;
}

int Titon::getOutsideTemp() {
  return data.t_outside.value;
}

int Titon::getIncomingTemp() {
  return data.t_incoming.value;
}

int Titon::getExhaustTemp() {
  return data.t_exhaust.value;
}

boolean Titon::isOn() {
  return data.is_on.value;
}

boolean Titon::isRhMode() {
  return data.is_rh_mode.value;
}

boolean Titon::isHeatingMode() {
  return data.is_heating_mode.value;
}

boolean Titon::isSwitchActive() {
  return data.is_switch_active.value;
}

boolean Titon::isSummerMode() {
  return data.is_summer_mode.value;
}

boolean Titon::isErrorRelay() {
  return data.is_error.value;
}

boolean Titon::isMotorIn() {
  return data.is_in_motor.value;
}

boolean Titon::isFrontHeating() {
  return data.is_front_heating.value;
}

boolean Titon::isMotorOut() {
  return data.is_out_motor.value;
}

boolean Titon::isHeating() {
  return data.is_heating.value;
}

boolean Titon::isFault() {
  return data.is_fault.value;
}

boolean Titon::isServiceNeeded() {
  return data.is_service.value;
}

int Titon::getServicePeriod() {
  return data.service_period.value;
}

int Titon::getServiceCounter() {
  return data.service_counter.value;
}

int Titon::getFanSpeed() {
  return data.fan_speed.value;
}

int Titon::getDefaultFanSpeed() {
  return data.default_fan_speed.value;
}

int Titon::getRh1() {
  if (!data.rh1.lastReceived) {
    return NOT_SET;
  }
  return data.rh1.value;
}

int Titon::getRh2() {
  if (!data.rh2.lastReceived) {
    return NOT_SET;
  }
  return data.rh2.value;
}

int Titon::getCO2() {
  if (!data.co2.lastReceived) {
    return NOT_SET;
  }
  return data.co2.value;
}

int Titon::getHeatingTarget() {
  return data.heating_target.value;
}

int Titon::getSwitchType() {
  if (!settings.is_boost_setting.lastReceived) {
    return NOT_SET;
  }
  return settings.is_boost_setting.value ? 1 : 0;
}

// private

// Requests
void Titon::sendInsideTempReq() {
  requestVariable(tn_VARIABLE_T_INSIDE);
}

void Titon::sendOutsideTempReq() {
  requestVariable(tn_VARIABLE_T_OUTSIDE);
}

void Titon::sendIncomingTempReq() {
  requestVariable(tn_VARIABLE_T_INCOMING);
}

void Titon::sendExhaustTempReq() {
  requestVariable(tn_VARIABLE_T_EXHAUST);
}

void Titon::sendStatusReq() {
  requestVariable(tn_VARIABLE_STATUS);
}

void Titon::sendServicePeriodReq() {
  requestVariable(tn_VARIABLE_SERVICE_PERIOD);
}

void Titon::sendFanSpeedReq() {
  requestVariable(tn_VARIABLE_FAN_SPEED);
}

void Titon::sendDefaultFanSpeedReq() {
  requestVariable(tn_VARIABLE_DEFAULT_FAN_SPEED);
}

void Titon::sendHeatingTargetReq() {
  requestVariable(tn_VARIABLE_HEATING_TARGET);
}

void Titon::sendIO08Req() {
  requestVariable(tn_VARIABLE_IO_08);
}

void Titon::sendFlags06Req() {
  requestVariable(tn_VARIABLE_FLAGS_06);
}

void Titon::sendProgramReq() {
  requestVariable(tn_VARIABLE_PROGRAM);
}

void Titon::sendServiceCounterReq() {
  requestVariable(tn_VARIABLE_SERVICE_COUNTER);
}

void Titon::sendRhReq() {
  requestVariable(tn_VARIABLE_RH1);
}

// set generic variable value in all mainboards and panels
void Titon::setVariable(byte variable, byte value) {
  setVariable(variable, value, tn_MSG_MAINBOARDS);
}

void Titon::setVariable(byte variable, byte value, byte target) {
  byte message[tn_MSG_LENGTH];
  message[0] = tn_MSG_DOMAIN;
  message[1] = tn_MSG_THIS_PANEL;
  message[2] = target;
  message[3] = variable;
  message[4] = value;
  message[5] = calculateCheckSum(message);

  // send to all mainboards
  for (int i = 0; i < tn_MSG_LENGTH; i++) {
    serial->write(message[i]);
  }

  if (isDebug && packetCallback) {
    // Callback that we got the message
    packetCallback(message, tn_MSG_LENGTH, (char*)"packetSent");
  }

  message[1] = tn_MSG_MAINBOARD_1;
  message[2] = tn_MSG_PANELS;
  message[5] = calculateCheckSum(message);

  // send to all panels
  for (int i = 0; i < tn_MSG_LENGTH; i++) {
    serial->write(message[i]);
  }
}

void Titon::requestVariable(byte variable) {
  byte message[tn_MSG_LENGTH];
  message[0] = tn_MSG_DOMAIN;
  message[1] = tn_MSG_THIS_PANEL;
  message[2] = tn_MSG_MAINBOARD_1;
  message[3] = tn_MSG_POLL_BYTE;
  message[4] = variable;
  message[5] = calculateCheckSum(message);


  if (isDebug && packetCallback) {
    // Callback that we got the message
    packetCallback(message, tn_MSG_LENGTH, (char*)"packetSent");
  }

  for (int i = 0; i < tn_MSG_LENGTH; i++) {
    serial->write(message[i]);
  }

  delay(100);
}

// tries to read one full message
// returns true if a message was read, false otherwise
boolean Titon::readMessage(byte message[]) {
  boolean ret = false;

  if (serial->available() >= tn_MSG_LENGTH) {
    message[0] = serial->read();

    if (message[0] == tn_MSG_DOMAIN) {
      message[1] = serial->read();
      message[2] = serial->read();

      // accept messages from mainboard 1 or panel 1
      // accept messages to panel 1, mainboard 1 or to all panels and mainboards
      if ((message[1] == tn_MSG_MAINBOARD_1 || message[1] == tn_MSG_THIS_PANEL || message[1] == tn_MSG_PANEL_1) &&
          (message[2] == tn_MSG_PANELS || message[2] == tn_MSG_THIS_PANEL || message[2] == tn_MSG_PANEL_1 ||
           message[2] == tn_MSG_MAINBOARD_1 || message[2] == tn_MSG_MAINBOARDS)) {
        int i = 3;
        // read the rest of the message
        while (i < tn_MSG_LENGTH) {
          message[i++] = serial->read();
        }

        if (isDebug && packetCallback) {
          // Callback that we got the message
          packetCallback(message, tn_MSG_LENGTH, (char*)"packetRecv");
        }

        ret = true;
      }
    }
  }

  return ret;
}

void Titon::decodeMessage(const byte message[]) {
  // decode variable in message
  byte variable = message[3];
  byte value = message[4];
  unsigned long now = millis();

  // Check message checksum
  if (!validateCheckSum(message)) {
    return ;// Message invalid
  }

  // Temperature (status object)
  if (variable == tn_VARIABLE_T_OUTSIDE) { // OUTSIDE
    checkValueChange(&(data.t_outside.value), ntc2Cel(value), &(data.t_outside.lastReceived));
  } else if (variable == tn_VARIABLE_T_EXHAUST) { // EXHAUST
    checkValueChange(&(data.t_exhaust.value), ntc2Cel(value), &(data.t_exhaust.lastReceived));
  } else if (variable == tn_VARIABLE_T_INSIDE) { // INSIDE
    checkValueChange(&(data.t_inside.value), ntc2Cel(value), &(data.t_inside.lastReceived));
  } else if (variable == tn_VARIABLE_T_INCOMING) { // INCOMING
    checkValueChange(&(data.t_incoming.value), ntc2Cel(value), &(data.t_incoming.lastReceived));
  }

  // RH
  else if (variable == tn_VARIABLE_RH1) { 
    checkValueChange(&(data.rh1.value), hex2Rh(value), &(data.rh1.lastReceived));
  } else if (variable == tn_VARIABLE_RH2) {
    checkValueChange(&(data.rh2.value), hex2Rh(value), &(data.rh2.lastReceived));
  }

  // CO2
  // Let's assume that the timeinterval for the same value is something pre-defined..
  else if (variable == tn_VARIABLE_CO2_HI) {
    data.co2_hi.lastReceived = millis();
    data.co2_hi.value = value;
    if (data.co2_lo.lastReceived > millis() - CO2_LIFE_TIME_MS) {
      handleCo2TotalValue(data.co2_hi.value, data.co2_lo.value);
    }
  } else if (variable == tn_VARIABLE_CO2_LO) {
    data.co2_lo.lastReceived = millis();
    data.co2_lo.value = value;
    if (data.co2_hi.lastReceived > millis() - CO2_LIFE_TIME_MS) {
      handleCo2TotalValue(data.co2_hi.value, data.co2_lo.value);
    }
  }

  // Others (config object)
  else if (variable == tn_VARIABLE_FAN_SPEED) {
    data.fan_speed.lastReceived = millis();
    checkStatusChange(&(data.fan_speed.value), hex2FanSpeed(value));
  } else if (variable == tn_VARIABLE_DEFAULT_FAN_SPEED) {
    data.default_fan_speed.lastReceived = millis();
    checkStatusChange(&(data.default_fan_speed.value), hex2FanSpeed(value));
  } else if (variable == tn_VARIABLE_STATUS) {
    decodeStatus(value);
  } else if (variable == tn_VARIABLE_IO_08) {
    decodeVariable08(value);
  } else if (variable == tn_VARIABLE_FLAGS_06) {
    decodeFlags06(value);
  } else if (variable == tn_VARIABLE_SERVICE_PERIOD) {
    data.service_period.lastReceived = millis();
    checkStatusChange(&(data.service_period.value), value);
  } else if (variable == tn_VARIABLE_SERVICE_COUNTER) {
    data.service_counter.lastReceived = millis();
    checkStatusChange(&(data.service_counter.value), value);
  } else if (variable == tn_VARIABLE_HEATING_TARGET) {
    data.heating_target.lastReceived = millis();
    checkStatusChange(&(data.heating_target.value), ntc2Cel(value));
  } else if (variable == tn_VARIABLE_PROGRAM) {
    decodeProgram(value);
  } else {
    // variable not recognized
  }

  if (!fullInitDone) { // Only send once after all the decoding has been successfully done
    fullInitDone = isStatusInitDone();
    if (fullInitDone) {
      statusChangedCallback(); // Inform only when full init is done to avoid non-set variables being presented
    }
  }
}

// For now, read only (no mutex needed)
void Titon::decodeVariable08(byte variable08) {
  // flags of variable 08
  unsigned long now = millis();

  data.is_summer_mode.lastReceived = now;
  data.is_error.lastReceived = now;
  data.is_in_motor.lastReceived = now;
  data.is_front_heating.lastReceived = now;
  data.is_out_motor.lastReceived = now;
  data.is_extra_func.lastReceived = now;

  data.variable08.value = variable08;
  data.variable08.lastReceived = now;

  checkStatusChange(&(data.is_summer_mode.value), (variable08 & tn_08_FLAG_SUMMER_MODE) != 0x00);
  checkStatusChange(&(data.is_error.value), (variable08 & tn_08_FLAG_ERROR_RELAY) != 0x00);
  checkStatusChange(&(data.is_in_motor.value), (variable08 & tn_08_FLAG_MOTOR_IN) != 0x00);
  checkStatusChange(&(data.is_front_heating.value), (variable08 & tn_08_FLAG_FRONT_HEATING) != 0x00);
  checkStatusChange(&(data.is_out_motor.value), (variable08 & tn_08_FLAG_MOTOR_OUT) != 0x00);
  checkStatusChange(&(data.is_extra_func.value), (variable08 & tn_08_FLAG_EXTRA_FUNC) != 0x00);
}

// For now, read only (no mutex needed)
void Titon::decodeFlags06(byte flags06) {
  // flags of variable 06
  unsigned long now = millis();
  data.is_switch_active.lastReceived = now;

  data.flags06.value = flags06;
  data.flags06.lastReceived = now;

  checkStatusChange(&(data.is_switch_active.value), (flags06 & tn_06_FIREPLACE_FLAG_IS_ACTIVE) != 0x00);
}

void Titon::decodeProgram(byte program) {
  // flags of programs variable
  bool shoudInformCallback = !settings.is_boost_setting.lastReceived;

  unsigned long now = millis();
  settings.is_boost_setting.lastReceived = now;

  settings.program.value = program;
  settings.program.lastReceived = now;

  checkSettingsChange(&(settings.is_boost_setting.value), (program & tn_PROGRAM_SWITCH_TYPE) != 0x00);

  if (shoudInformCallback) {
    // Never received, publish
    statusChangedCallback();
  }
}

void Titon::decodeStatus(byte status) {
  unsigned long now = millis();

  data.is_on.lastReceived = now;
  data.is_rh_mode.lastReceived = now;
  data.is_heating_mode.lastReceived = now;
  data.is_filter.lastReceived = now;
  data.is_heating.lastReceived = now;
  data.is_fault.lastReceived = now;
  data.is_service.lastReceived = now;

  data.status.value = status; // This is the full data status
  data.status.lastReceived = now;

  checkStatusChange(&(data.is_on.value), (status & tn_STATUS_FLAG_POWER) != 0x00);
  checkStatusChange(&(data.is_rh_mode.value), (status & tn_STATUS_FLAG_RH) != 0x00);
  checkStatusChange(&(data.is_heating_mode.value), (status & tn_STATUS_FLAG_HEATING_MODE) != 0x00);
  checkStatusChange(&(data.is_filter.value), (status & tn_STATUS_FLAG_FILTER) != 0x00);
  checkStatusChange(&(data.is_heating.value), (status & tn_STATUS_FLAG_HEATING) != 0x00);
  checkStatusChange(&(data.is_fault.value), (status & tn_STATUS_FLAG_FAULT) != 0x00);
  checkStatusChange(&(data.is_service.value), (status & tn_STATUS_FLAG_SERVICE) != 0x00);

  statusMutex = false; // Clear the status mutex, allow to continue
}

//
// Settings
void Titon::checkSettingsChange(boolean* oldValue, boolean newValue) {
  if (checkChange(oldValue, newValue)) {
    statusChangedCallback();
  }
}

//
// Status
void Titon::checkStatusChange(boolean* oldValue, boolean newValue) {
  if (checkChange(oldValue, newValue) && fullInitDone) {
    statusChangedCallback();
  }
}

void Titon::checkStatusChange(int* oldValue, int newValue) {
  if (checkChange(oldValue, newValue) && fullInitDone) {
    statusChangedCallback();
  }
}

//
// Temperature change
void Titon::checkValueChange(int *oldValue, int newValue, unsigned long *lastReceived) {
  unsigned long now = millis();

  *lastReceived = now;
  checkValueChange(oldValue, newValue);
}

// Check for value change (Temperature, CO2, RH)
void Titon::checkValueChange(int* oldValue, int newValue) {
  if (checkChange(oldValue, newValue) && isTemperatureInitDone()) { // Do not publish status, until base values has been received
    temperatureChangedCallback();
  }
}


int Titon::ntc2Cel(byte ntc) {
  int i = (int)ntc;
  return tnTemps[i];
}

byte Titon::cel2Ntc(int cel) {
  for (int i = 0; i < 256; i++) {
    if (tnTemps[i] == cel) {
      return i;
    }
  }

  // we should not be here, return 10 Cel as default
  return 0x83;
}

byte Titon::fanSpeed2Hex(int fan) {
  if (fan > 0 && fan < 9) {
    return tnFanSpeeds[fan - 1];
  }

  // we should not be here, return speed 1 as default
  return tn_FAN_SPEED_1;
}

int Titon::hex2FanSpeed(byte hex) {
  for (int i = 0; i < sizeof(tnFanSpeeds); i++) {
    if (tnFanSpeeds[i] == hex) {
      return i + 1;
    }
  }

  return NOT_SET;
}

int Titon::hex2Rh(byte hex) {
  if (hex >= 51) {
    return (hex - 51) / 2.04;
  } else {
    return NOT_SET;
  }
}

byte Titon::htCel2Hex(int htCel) {
  if (htCel < 13) {
    return 0x01;
  } else if (htCel < 15) {
    return 0x03;
  } else if (htCel < 18) {
    return 0x07;
  } else if (htCel < 20) {
    return 0x0F;
  } else if (htCel < 23) {
    return 0x1F;
  } else if (htCel < 25) {
    return 0x3F;
  } else if (htCel < 27) {
    return 0x7F;
  } else if (htCel == 27) {
    return 0xFF;
  } else {
    return 0x01;
  }
}

// calculate tn message checksum
byte Titon::calculateCheckSum(const byte message[]) {
  byte ret = 0x00;
  for (int i = 0; i < tn_MSG_LENGTH - 1; i++) {
    ret += message[i];
  }

  return ret;
}

bool Titon::validateCheckSum(const byte message[]) {
  byte calculated = calculateCheckSum(message); // Calculated check sum
  byte received = message[5];

  if (calculated != received) {
    debugPrintCallback("Checksum comparison failed!");
    return false;
  }

  return true;
}


unsigned long Titon::checkChange(boolean* oldValue, boolean newValue) {
  unsigned long changed = 0;

  if (*oldValue != newValue) {
    *oldValue = newValue;
    data.updated = millis();
    changed = data.updated;
  }

  return changed;
}

unsigned long Titon::checkChange(int* oldValue, int newValue) {
  unsigned long changed = 0;
  if (*oldValue != newValue) {
    *oldValue = newValue;
    data.updated = millis();
    changed = data.updated;
  }

  return changed;
}

void Titon::retryLoop() {
  sendMissingRequests();
  statusMutex = false; // Clear the status mutex (prevents possible deadlocks of status)
}

void Titon::sendMissingRequests() {
  if (!data.is_on.lastReceived) sendStatusReq();
  if (!data.variable08.lastReceived) sendIO08Req();
  if (!data.fan_speed.lastReceived) sendFanSpeedReq();
  if (!data.default_fan_speed.lastReceived) sendDefaultFanSpeedReq();
  if (!data.service_period.lastReceived) sendServicePeriodReq();
  if (!data.service_counter.lastReceived) sendServiceCounterReq();
  if (!data.heating_target.lastReceived) sendHeatingTargetReq();
}

boolean Titon::isTemperatureInitDone() {
  return data.t_outside.lastReceived &&
         data.t_inside.lastReceived &&
         data.t_exhaust.lastReceived &&
         data.t_incoming.lastReceived;
}

boolean Titon::isStatusInitDone() { // all initializations
  // Ensure that all data values has been received
  return
    data.is_on.lastReceived &&
    data.is_rh_mode.lastReceived &&
    data.is_heating_mode.lastReceived &&
    data.variable08.lastReceived &&
    data.is_filter.lastReceived &&
    data.is_heating.lastReceived &&
    data.is_fault.lastReceived &&
    data.is_service.lastReceived &&
    data.fan_speed.lastReceived &&
    data.default_fan_speed.lastReceived &&
    data.service_period.lastReceived &&
    data.service_counter.lastReceived &&
    data.heating_target.lastReceived;
}

void Titon::handleCo2TotalValue(byte hi, byte low) {
  // Construct co2 value from hi and lo bytes
  uint16_t total = low + (hi << 8);
  checkValueChange(&(data.co2.value), total, &(data.co2.lastReceived));
}

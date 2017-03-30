#ifdef ENCODER_ON
void encoderInterrupts() {
  attachInterrupt(digitalPinToInterrupt(atoi(JConf.encoder_pin_a)), encoderReadPinA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(atoi(JConf.encoder_pin_b)), encoderReadPinB, CHANGE);
}

void encoderSetup() {
  pinMode(atoi(JConf.encoder_pin_a), INPUT_PULLUP);
  pinMode(atoi(JConf.encoder_pin_b), INPUT_PULLUP);
  encoderInterrupts();
  encoderResetTimer = timer.setInterval(encoderResetInterval, encoderReset);
}

void encoderReset() {
  if (!encoderFlagA && !encoderFlagB) return;
  encoderDirection = 0;
  encoderFlagA = false;
  encoderFlagB = false;
  timer.restartTimer(encoderResetTimer);
  encoderInterrupts();
  addLog_P(LOG_LEVEL_DEBUG, "encoderReset()");
}

void encoderAction(bool reverse = false) {
  if (reverse == true) {
    addLog_P(LOG_LEVEL_ERROR, "Encoder <= Reverse direction");
  } else {
    addLog_P(LOG_LEVEL_ERROR, "Encoder => Forward direction");
  }
  encoderReset();
}

void encoderReadPinA() {
  if (encoderFlagA == false) {
    if (digitalRead(atoi(JConf.encoder_pin_b)) == LOW) return;
    addLog_P(LOG_LEVEL_ERROR, "Flag A");
    encoderFlagA = true;
    detachInterrupt(atoi(JConf.encoder_pin_a));
    if (encoderDirection == 0) {
      encoderDirection = 1;
    } else if (encoderDirection == 2) {
      encoderAction(true); //Reverse direction
    }
  }
}

void encoderReadPinB() {
  if (encoderFlagB == false) {
    if (digitalRead(atoi(JConf.encoder_pin_a)) == LOW) return;
    addLog_P(LOG_LEVEL_ERROR, "Flag B");
    encoderFlagB = true;
    detachInterrupt(atoi(JConf.encoder_pin_b));
    if (encoderDirection == 0) {
      encoderDirection = 2;
    } else if (encoderDirection == 1) {
      encoderAction();
    }
  }
}
#endif //ENCODER_ON

#ifdef BH1750_ON
void GetLightSensorData() {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: GetLightSensorData Start");

  luxString = String(lightSensor.readLightLevel());

  snprintf_P(log, sizeof(log), PSTR("GetLightSensorData: Lux: %s"), luxString.c_str());
  addLog(LOG_LEVEL_INFO, log);

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: GetLightSensorData load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}
#endif

#ifdef BME280_ON
void GetBmeSensorData() {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: GetBmeSensorData Start");

  temperatureString = String(bmeSensor.readTempC());
  snprintf_P(log, sizeof(log), PSTR("GetBmeSensorData: Temperature: %s C"), temperatureString.c_str());
  addLog(LOG_LEVEL_INFO, log);

  pressureString = String(bmeSensor.readFloatPressure() / 133.3F);
  snprintf_P(log, sizeof(log), PSTR("GetBmeSensorData: Pressure: %s"), pressureString.c_str());
  addLog(LOG_LEVEL_INFO, log);

  humidityString = String(bmeSensor.readFloatHumidity());
  snprintf_P(log, sizeof(log), PSTR("GetBmeSensorData: Humidity: %s %"), humidityString.c_str());
  addLog(LOG_LEVEL_INFO, log);

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: GetBmeSensorData load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}
#endif

#ifdef SHT21_ON
void GetSHT21SensorData() {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: GetSHT21SensorData Start");

  myHTU21D.setResolution(HTU21D_RES_RH8_TEMP12);

  temperatureString = String(myHTU21D.readTemperature());
  snprintf_P(log, sizeof(log), PSTR("GetSHT21SensorData: Temperature: %s C"), temperatureString.c_str());
  addLog(LOG_LEVEL_INFO, log);

  humidityString = String(myHTU21D.readCompensatedHumidity());
  snprintf_P(log, sizeof(log), PSTR("GetSHT21SensorData: Humidity: %s %"), humidityString.c_str());
  addLog(LOG_LEVEL_INFO, log);

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: GetSHT21SensorData load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}
#endif

#ifdef DHT_ON
void GetDhtSensorData() {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: GetDhtSensorData Start");

  float humidityData = dht.readHumidity();
  float temperatureData = dht.readTemperature();

  if (isnan(humidityData) || isnan(temperatureData)) {
    addLog_P(LOG_LEVEL_ERROR, "GetDhtSensorData: Error reading DHT!");
    return;
  } else {
    temperatureString = String(temperatureData);
    snprintf_P(log, sizeof(log), PSTR("GetDhtSensorData: Temperature: %s C"), temperatureString.c_str());
    addLog(LOG_LEVEL_INFO, log);

    humidityString = String(humidityData);
    snprintf_P(log, sizeof(log), PSTR("GetDhtSensorData: Humidity: %s %"), humidityString.c_str());
    addLog(LOG_LEVEL_INFO, log);

  }

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: GetDhtSensorData load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}
#endif

#ifdef DS18X20_ON
void SearchDS18x20Sensors() {
  if (searchDsSensorDone) {
    return;
  }

  char log[LOGSZ];
  if (!ds.search(dsData[currentDsSensor].address)) {
    searchDsSensorDone = true;
    currentDsSensor = 0;
    ds.reset_search();
    MqttInitDS();
    return;
  } else if (!searchDsSensorDone) {
    findDsSensors ++;
  }

  if (OneWire::crc8(dsData[currentDsSensor].address, 7) != dsData[currentDsSensor].address[7]) {
    addLog_P(LOG_LEVEL_ERROR, "DS Sensor Address CRC is not valid!");
    return;
  }

  String addr = "";
  for (size_t i = 0; i < 8; i++) {
    addr += String(dsData[currentDsSensor].address[i], HEX);
  }
  dsData[currentDsSensor].addressString = addr;
  switch (dsData[currentDsSensor].address[0]) {
    case 0x10:
      dsData[currentDsSensor].type = DS18S20;
      break;
    case 0x28:
      dsData[currentDsSensor].type = DS18B20;
      break;
    case 0x22:
      dsData[currentDsSensor].type = DS1822;
      break;
    default:
      dsData[currentDsSensor].type = UNKNOWN;
      return;
  }
  currentDsSensor ++;

  snprintf_P(log, sizeof(log), PSTR("DS: currentDsSensor:%d  findDsSensors:%d"), currentDsSensor, findDsSensors);
  addLog(LOG_LEVEL_INFO, log);
  SearchDS18x20Sensors();
}

void GetDS18x20SensorData() {
  if (findDsSensors == 0) {
    addLog_P(LOG_LEVEL_ERROR, "DS Sensors Not Found!");
    searchDsSensorDone = false;
    currentDsSensor = 0;
    return;
  }

  byte i;
  byte data[12];

  if (!flag_ds_sensor_read_delay) {
    flag_ds_sensor_read_delay = true;
    ds.reset();
    ds.select(dsData[currentDsSensor].address);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end
    timer.setTimeout(800, GetDS18x20SensorData);
    return;
  } else {
    flag_ds_sensor_read_delay = false;
  }

  ds.reset();
  ds.select(dsData[currentDsSensor].address);
  ds.write(0xBE);         // Read Scratchpad

  for (i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  if (OneWire::crc8(data, 8) != data[8]) {
    addLog_P(LOG_LEVEL_ERROR, "DS Sensor: Data CRC is not valid!");
    dsData[currentDsSensor].dsTemp = "none";
    nextDsSensor();
    return;
  }
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (dsData[currentDsSensor].type == DS18S20) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else if (dsData[currentDsSensor].type == DS18B20 || dsData[currentDsSensor].type == DS1822) {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  } else {
    addLog_P(LOG_LEVEL_ERROR, "Device is not a DS18x20 family device!");
  }

  dsData[currentDsSensor].dsTemp = String((float) raw / 16.0);
  dsDataPrint();
  nextDsSensor();
}

void nextDsSensor() {
  if (findDsSensors == currentDsSensor + 1) {
    currentDsSensor = 0;
  } else {
    currentDsSensor ++;
  }
}

void dsDataPrint() {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: dsDataPrint Start");

  String dsType = " ";

  switch (dsData[currentDsSensor].address[0]) {
    case 0x10:
      dsType = "DS18S20";  // or old DS1820
      break;
    case 0x28:
      dsType = "DS18B20";
      break;
    case 0x22:
      dsType = "DS1822";
      break;
    default:
      addLog_P(LOG_LEVEL_ERROR, "Device is not a DS18x20 family device!");
      return;
  }

  snprintf_P(log, sizeof(log), PSTR("DS type:%s  addr:%s  temp:%sC"), dsType.c_str(), dsData[currentDsSensor].addressString.c_str(), dsData[currentDsSensor].dsTemp.c_str());
  addLog(LOG_LEVEL_INFO, log);

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: dsDataPrint load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}
#endif //DS18X20_ON

#if defined(PZEM_ON)
bool GetPzemData(float data, String * val) {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: GetPzemData Start");

  if (data < 0.0) {
    addLog_P(LOG_LEVEL_ERROR, "GetPzemData: Error reading data!");
    pzem.setAddress(ip_pzem);
    pzem.setReadTimeout(500);
    return false;
  } else if (pzem_current_read == PZEM_POWER || pzem_current_read == PZEM_ENERGY) {
    data = data * coil_ratio / 1000;
  } else if (pzem_current_read == PZEM_CURRENT) {
    data = data * coil_ratio;
  }
  *val = String(data);

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: GetPzemData load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);

  return true;
}

void GetPzemSerialRead() {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: GetPzemSerialRead Start");

  switch (pzem_current_read) {
    case PZEM_VOLTAGE:
      if (GetPzemData(pzem.voltage(ip_pzem), &pzemVoltageString)) {
        snprintf_P(log, sizeof(log), PSTR("GetPzemSerialRead: Voltage: %s V"), pzemVoltageString.c_str());
        addLog(LOG_LEVEL_INFO, log);
        pzem_current_read = PZEM_CURRENT;
      }
      break;
    case PZEM_CURRENT:
      if (GetPzemData(pzem.current(ip_pzem), &pzemCurrentString)) {
        snprintf_P(log, sizeof(log), PSTR("GetPzemSerialRead: Current: %s A"), pzemCurrentString.c_str());
        addLog(LOG_LEVEL_INFO, log);
        pzem_current_read = PZEM_POWER;
      }
      break;
    case PZEM_POWER:
      if (GetPzemData(pzem.power(ip_pzem), &pzemPowerString)) {
        snprintf_P(log, sizeof(log), PSTR("GetPzemSerialRead: Power: %s W"), pzemPowerString.c_str());
        addLog(LOG_LEVEL_INFO, log);
        pzem_current_read = PZEM_ENERGY;
      }
      break;
    case PZEM_ENERGY:
      if (GetPzemData(pzem.energy(ip_pzem), &pzemEnergyString)) {
        snprintf_P(log, sizeof(log), PSTR("GetPzemSerialRead: Energy: %s Wh"), pzemEnergyString.c_str());
        addLog(LOG_LEVEL_INFO, log);
      }
      pzem_current_read = PZEM_VOLTAGE;
      break;
    default:
      pzem_current_read = PZEM_VOLTAGE;
  }

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: GetPzemSerialRead load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}

void PzemResetEnergy() {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: PzemResetEnergy Start");

  switch (pzem_reset_stage) {
    case PZEM_STAGE1:
      light1State = ON;
      Light1Control();
      pzem_reset_stage = PZEM_STAGE2;
      timer.setTimeout(6000, PzemResetEnergy);
      break;
    case PZEM_STAGE2:
      light1State = OFF;
      Light1Control();
      pzem_reset_stage = PZEM_STAGE3;
      timer.setTimeout(1000, PzemResetEnergy);
      break;
    case PZEM_STAGE3:
      light1State = ON;
      Light1Control();
      pzem_reset_stage = PZEM_STAGE4;
      timer.setTimeout(1000, PzemResetEnergy);
      break;
    case PZEM_STAGE4:
      light1State = OFF;
      Light1Control();
      pzem_reset_stage = PZEM_STAGE1;
      break;
    default:
      break;
  }

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: PzemResetEnergy load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}
#endif

#ifdef MHZ19_ON
int GetMHZ19() {
  char log[LOGSZ];

  // command to ask for data
  byte cmd[RESPONSE_SIZE] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  char data[RESPONSE_SIZE];

  Serial.write(cmd, RESPONSE_SIZE); //request PPM CO2

  unsigned long startTime = millis();
  uint8_t len = 0;
  while ((len < RESPONSE_SIZE) && (millis() - startTime < READ_TIMEOUT)) {
    if (Serial.available() > 0) {
      Serial.readBytes(data, RESPONSE_SIZE);
    }
  }

  if (data[0] != 0xFF) {
    addLog_P(LOG_LEVEL_ERROR, "Wrong starting byte from co2 sensor!");
    return -1;
  }

  if (data[1] != 0x86) {
    addLog_P(LOG_LEVEL_ERROR, "Wrong command from co2 sensor!");
    return -1;
  }

  int responseHigh = (int) data[2];
  int responseLow = (int) data[3];
  int ppm = (256 * responseHigh) + responseLow;
  mhz19PpmString = String(ppm);

  snprintf_P(log, sizeof(log), PSTR("GetMHZ19: CO2: %d PPM"), ppm);
  addLog(LOG_LEVEL_INFO, log);
  return ppm;
}
#endif // MHZ19_ON

//========= MQTT ========
bool MqttConnect() {
  if (atoi(JConf.mqtt_enable) != 1) {
    return false;
  }

  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: MqttConnect Start");

  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return true;
  }

  addLog_P(LOG_LEVEL_INFO, "MqttConnect: Connecting to MQTT...");
  if ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    snprintf_P(log, sizeof(log), PSTR("MqttConnect: Error: %s"), String(mqtt.connectErrorString(ret)).c_str());
    //        snprintf_P(log, sizeof(log), String(mqtt.connectErrorString(ret)).c_str());

    addLog(LOG_LEVEL_ERROR, log);
    mqtt.disconnect();
    return false;
  }

  addLog_P(LOG_LEVEL_INFO, "MqttConnect: MQTT Connected");

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: MqttConnect load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);

  return true;
}

void MqttInit() {
  if (atoi(JConf.mqtt_enable) != 1) {
    return;
  }
  //Publish Topics
  sprintf(Light1State_buff, "%s%s%s", JConf.publish_topic, Light1State, JConf.mqtt_name);
  pubTopicLight1State = Adafruit_MQTT_Publish(&mqtt, Light1State_buff);
  sprintf(Light2State_buff, "%s%s%s", JConf.publish_topic, Light2State, JConf.mqtt_name);
  pubTopicLight2State = Adafruit_MQTT_Publish(&mqtt, Light2State_buff);

  sprintf(Light1StartTime_buff, "%s%s%s", JConf.publish_topic, Light1StartTime, JConf.mqtt_name);
  pubTopicLight1StartTime = Adafruit_MQTT_Publish(&mqtt, Light1StartTime_buff);
  sprintf(Light1StopTime_buff, "%s%s%s", JConf.publish_topic, Light1StopTime, JConf.mqtt_name);
  pubTopicLight1StopTime = Adafruit_MQTT_Publish(&mqtt, Light1StopTime_buff);

  sprintf(Light2StartTime_buff, "%s%s%s", JConf.publish_topic, Light2StartTime, JConf.mqtt_name);
  pubTopicLight2StartTime = Adafruit_MQTT_Publish(&mqtt, Light2StartTime_buff);
  sprintf(Light2StopTime_buff, "%s%s%s", JConf.publish_topic, Light2StopTime, JConf.mqtt_name);
  pubTopicLight2StopTime = Adafruit_MQTT_Publish(&mqtt, Light2StopTime_buff);

  sprintf(motionSensor_buff, "%s%s%s", JConf.publish_topic, motionSensor, JConf.mqtt_name);
  pubTopicMotionSensor = Adafruit_MQTT_Publish(&mqtt, motionSensor_buff);

  sprintf(lux_buff, "%s%s%s", JConf.publish_topic, lux, JConf.mqtt_name);
  pubTopicLux = Adafruit_MQTT_Publish(&mqtt, lux_buff);

  sprintf(temperature_buff, "%s%s%s", JConf.publish_topic, temperature, JConf.mqtt_name);
  pubTopicTemperature = Adafruit_MQTT_Publish(&mqtt, temperature_buff);

  sprintf(humidity_buff, "%s%s%s", JConf.publish_topic, humidity, JConf.mqtt_name);
  pubTopicHumidity = Adafruit_MQTT_Publish(&mqtt, humidity_buff);

  sprintf(pressure_buff, "%s%s%s", JConf.publish_topic, pressure, JConf.mqtt_name);
  pubTopicPressure = Adafruit_MQTT_Publish(&mqtt, pressure_buff);

#ifdef PZEM_ON
  sprintf(pzemVoltage_buff, "%s%s%s", JConf.publish_topic, pzemVoltage, JConf.mqtt_name);
  pubTopicPzemVoltage = Adafruit_MQTT_Publish(&mqtt, pzemVoltage_buff);

  sprintf(pzemCurrent_buff, "%s%s%s", JConf.publish_topic, pzemCurrent, JConf.mqtt_name);
  pubTopicPzemCurrent = Adafruit_MQTT_Publish(&mqtt, pzemCurrent_buff);

  sprintf(pzemPower_buff, "%s%s%s", JConf.publish_topic, pzemPower, JConf.mqtt_name);
  pubTopicPzemPower = Adafruit_MQTT_Publish(&mqtt, pzemPower_buff);

  sprintf(pzemEnergy_buff, "%s%s%s", JConf.publish_topic, pzemEnergy, JConf.mqtt_name);
  pubTopicPzemEnergy = Adafruit_MQTT_Publish(&mqtt, pzemEnergy_buff);
#endif //PZEM_ON

  sprintf(mhz19ppm_buff, "%s%s%s", JConf.publish_topic, mhz19ppm, JConf.mqtt_name);
  pubTopicMhz19ppm = Adafruit_MQTT_Publish(&mqtt, mhz19ppm_buff);

  sprintf(freeMemory_buff, "%s%s%s", JConf.publish_topic, freeMemory, JConf.mqtt_name);
  pubTopicFreeMemory = Adafruit_MQTT_Publish(&mqtt, freeMemory_buff);

  sprintf(uptime_buff, "%s%s%s", JConf.publish_topic, uptime, JConf.mqtt_name);
  pubTopicUptime = Adafruit_MQTT_Publish(&mqtt, uptime_buff);

  sprintf(version_buff, "%s%s%s", JConf.publish_topic, version, JConf.mqtt_name);
  pubTopicVersion = Adafruit_MQTT_Publish(&mqtt, version_buff);

  sprintf(ip_buff, "%s%s%s", JConf.publish_topic, ip, JConf.mqtt_name);
  pubTopicIp = Adafruit_MQTT_Publish(&mqtt, ip_buff);

  sprintf(mac_buff, "%s%s%s", JConf.publish_topic, mac, JConf.mqtt_name);
  pubTopicMac = Adafruit_MQTT_Publish(&mqtt, mac_buff);

  //Subscribe Topics
  sprintf(Light1State_buff_sub, "%s%s%s", JConf.subscribe_topic, Light1State, JConf.mqtt_name);
  subTopicLight1State = Adafruit_MQTT_Subscribe(&mqtt, Light1State_buff_sub);
  sprintf(Light2State_buff_sub, "%s%s%s", JConf.subscribe_topic, Light2State, JConf.mqtt_name);
  subTopicLight2State = Adafruit_MQTT_Subscribe(&mqtt, Light2State_buff_sub);

  sprintf(Light1StartTime_buff_sub, "%s%s%s", JConf.subscribe_topic, Light1StartTime, JConf.mqtt_name);
  subTopicLight1StartTime = Adafruit_MQTT_Subscribe(&mqtt, Light1StartTime_buff_sub);
  sprintf(Light1StopTime_buff_sub, "%s%s%s", JConf.subscribe_topic, Light1StopTime, JConf.mqtt_name);
  subTopicLight1StopTime = Adafruit_MQTT_Subscribe(&mqtt, Light1StopTime_buff_sub);

  sprintf(Light2StartTime_buff_sub, "%s%s%s", JConf.subscribe_topic, Light2StartTime, JConf.mqtt_name);
  subTopicLight2StartTime = Adafruit_MQTT_Subscribe(&mqtt, Light2StartTime_buff_sub);
  sprintf(Light2StopTime_buff_sub, "%s%s%s", JConf.subscribe_topic, Light2StopTime, JConf.mqtt_name);
  subTopicLight2StopTime = Adafruit_MQTT_Subscribe(&mqtt, Light2StopTime_buff_sub);

  sprintf(uptime_buff_sub, "%s%s%s", JConf.subscribe_topic, uptime, JConf.mqtt_name);
  subTopicUptime = Adafruit_MQTT_Subscribe(&mqtt, uptime_buff_sub);

  sprintf(saveConfig_buff_sub, "%s%s%s", JConf.subscribe_topic, saveConfig, JConf.mqtt_name);
  subTopicSaveConfig = Adafruit_MQTT_Subscribe(&mqtt, saveConfig_buff_sub);

#ifdef PZEM_ON
  sprintf(pzemReset_buff_sub, "%s%s%s", JConf.subscribe_topic, pzemReset, JConf.mqtt_name);
  subTopicPzemReset = Adafruit_MQTT_Subscribe(&mqtt, pzemReset_buff_sub);
#endif //PZEM_ON
}

#ifdef DS18X20_ON
void MqttInitDS() {
  if (atoi(JConf.mqtt_enable) != 1) {
    return;
  }

  for (size_t i = 0; i < findDsSensors; i++) {
    if (i == MAX_DS_SENSORS) break;
    sprintf(dsData[i].ds_buff, "%s%s%s", JConf.publish_topic, dsData[i].addressString.c_str(), JConf.mqtt_name);
    dsData[i].pubTopic = Adafruit_MQTT_Publish(&mqtt, dsData[i].ds_buff);
  }
}
#endif //DS18X20_ON

bool MqttPubLightState() {
  if (atoi(JConf.mqtt_enable) != 1) {
    return false;
  }

  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: MqttPubLightState Start");

  if (!mqtt.connected()) {
    addLog_P(LOG_LEVEL_ERROR, "MqttPubLightState: MQTT not connected!");
    return false;
  }

  String lightStateNum;
  if (light1State == ON) {
    lightStateNum = String(F("1"));
  } else if (light1State == OFF) {
    lightStateNum = String(F("0"));
  } else {
    lightStateNum = String(F("2"));
  }
  pubTopicLight1State.publish(lightStateNum.c_str());

  if (light2State == ON) {
    lightStateNum = String(F("1"));
  } else if (light2State == OFF) {
    lightStateNum = String(F("0"));
  } else {
    lightStateNum = String(F("2"));
  }
  pubTopicLight2State.publish(lightStateNum.c_str());

  pubTopicLight1StartTime.publish(JConf.light1_start_time);
  pubTopicLight1StopTime.publish(JConf.light1_stop_time);
  pubTopicLight2StartTime.publish(JConf.light2_start_time);
  pubTopicLight2StopTime.publish(JConf.light2_stop_time);

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: MqttPubLightState load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);

  return true;
}

bool MqttPublight1_offDelay() {
  if (atoi(JConf.mqtt_enable) != 1) {
    return false;
  }

  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: MqttPubLightState Start");

  if (!mqtt.connected()) {
    addLog_P(LOG_LEVEL_ERROR, "MqttPublight1_offDelay: MQTT not connected!");
    return false;
  }

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: MqttPublight1_offDelay load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);

  return true;
}

bool MqttPubData() {
  if (atoi(JConf.mqtt_enable) != 1) {
    return false;
  }

  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: MqttPubData Start");

  if (!mqtt.connected()) {
    addLog_P(LOG_LEVEL_ERROR, "MqttPubData: MQTT not connected!");
    return false;
  }

  if (atoi(JConf.bh1750_enable) == 1) {
    pubTopicLux.publish(luxString.c_str());
  }

  if (atoi(JConf.bme280_enable) == 1  ||  atoi(JConf.sht21_enable) == 1 ||  atoi(JConf.dht_enable) == 1) {
    pubTopicTemperature.publish(temperatureString.c_str());
    pubTopicHumidity.publish(humidityString.c_str());
  }

  if (atoi(JConf.bme280_enable) == 1) {
    pubTopicPressure.publish(pressureString.c_str());
  }

  pubTopicFreeMemory.publish(freeMemoryString.c_str());
  pubTopicUptime.publish(uptimeString.c_str());
  pubTopicVersion.publish(ver);
  pubTopicIp.publish(ipString.c_str());
  pubTopicMac.publish(macString.c_str());

#ifdef PZEM_ON
  if (atoi(JConf.pzem_enable) == 1) {
    pubTopicPzemVoltage.publish(pzemVoltageString.c_str());
    pubTopicPzemCurrent.publish(pzemCurrentString.c_str());
    pubTopicPzemPower.publish(pzemPowerString.c_str());
    pubTopicPzemEnergy.publish(pzemEnergyString.c_str());
  }
#endif

#ifdef MHZ19_ON
  if (atoi(JConf.mhz19_enable) == 1) {
    pubTopicMhz19ppm.publish(mhz19PpmString.c_str());
  }
#endif

#ifdef DS18X20_ON
  if (atoi(JConf.ds18x20_enable) == 1) {
    for (size_t i = 0; i < findDsSensors; i++) {
      if (i == MAX_DS_SENSORS) break;
      dsData[i].pubTopic.publish(dsData[i].dsTemp.c_str());
    }
  }
#endif //DS18X20_ON

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: MqttPubData load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);

  return true;
}

void CallbackLight1State(char *data, uint16_t len) {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: CallbackLight1State Start");

  if (strncmp (data, "1", 1) == 0) {
    light1State = ON;
  } else if (strncmp (data, "0", 1) == 0) {
    light1State = OFF;
  } else if (strncmp (data, "2", 1) == 0) {
    light1State = AUTO;
  }

  Light1Control();

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: CallbackLight1State load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}

void CallbackLight2State(char *data, uint16_t len) {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: CallbackLight2State Start");

  if (strncmp (data, "1", 1) == 0) {
    light2State = ON;
  } else if (strncmp (data, "0", 1) == 0) {
    light2State = OFF;
  } else if (strncmp (data, "2", 1) == 0) {
    light2State = AUTO;
  }

  Light2Control();

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: CallbackLight2State load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}

void CallbackLight1StartTime(char *data, uint16_t len) {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: CallbackLight1StartTime Start");

  strlcpy(JConf.light1_start_time, data, sizeof(JConf.light1_start_time));
  WorkTimeSettingsUpdate();

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: CallbackLight1State load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}

void CallbackLight2StartTime(char *data, uint16_t len) {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: CallbackLight2StartTime Start");

  strlcpy(JConf.light2_start_time, data, sizeof(JConf.light2_start_time));
  WorkTimeSettingsUpdate();

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: CallbackLight2StartTime load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}

void CallbackLight1StopTime(char *data, uint16_t len) {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: CallbackLight1StopTime Start");
  addLog_P(LOG_LEVEL_DEBUG, "Light1StopTime data: ");
  addLog_P(LOG_LEVEL_DEBUG, data);

  //  if (len == 6)
  //    if (IsDigit(data[0]) && IsDigit(data[1]) && IsDigit(data[3]) && IsDigit(data[3]) && data[2] == ':') {
  strlcpy(JConf.light1_stop_time, data, sizeof(JConf.light1_stop_time));
  WorkTimeSettingsUpdate();
  //    }

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: CallbackLight1StopTime load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}

void CallbackLight2StopTime(char *data, uint16_t len) {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: CallbackLight2StopTime Start");

  strlcpy(JConf.light2_stop_time, data, sizeof(JConf.light2_stop_time));
  WorkTimeSettingsUpdate();

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: CallbackLight2StopTime load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}

#ifdef PZEM_ON
void CallbackPzemReset(char *data, uint16_t len) {

  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: CallbackPzemReset Start");

  if (strncmp (data, "ON", 1) == 0) {
    PzemResetEnergy();
  }

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: CallbackPzemReset load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}
#endif

void CallbackUptime(char *data, uint16_t len) {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: CallbackUptime Start");

  timer.restartTimer(subscribeTimer);

#ifdef REBOOT_ON
  timer.restartTimer(rebootTimer);
#endif

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: CallbackUptime load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}

void CallbackSaveConfig(char *data, uint16_t len) {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: CallbackSaveConfig Start");

  //  sprintf_P(JConf.light2_off_delay, (const char *)F("%s"), data);
  JConf.saveConfig();

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: CallbackUptime load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}

void MqttSubscribe() {
  if (atoi(JConf.mqtt_enable) != 1) {
    return;
  }

  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: MqttSubscribe Start");

  subTopicLight1State.setCallback(CallbackLight1State);
  subTopicLight2State.setCallback(CallbackLight2State);
  subTopicLight1StartTime.setCallback(CallbackLight1StartTime);
  subTopicLight1StopTime.setCallback(CallbackLight1StopTime);
  subTopicLight2StartTime.setCallback(CallbackLight2StartTime);
  subTopicLight2StopTime.setCallback(CallbackLight2StopTime);
  subTopicUptime.setCallback(CallbackUptime);
  subTopicSaveConfig.setCallback(CallbackSaveConfig);

  //
  // Adafruit_MQTT.h define MAXSUBSCRIPTIONS 8
  //

  mqtt.subscribe(&subTopicLight1State);
  mqtt.subscribe(&subTopicLight2State);
  mqtt.subscribe(&subTopicLight1StartTime);
  mqtt.subscribe(&subTopicLight1StopTime);
  mqtt.subscribe(&subTopicLight2StartTime);
  mqtt.subscribe(&subTopicLight2StopTime);
  mqtt.subscribe(&subTopicUptime);
  mqtt.subscribe(&subTopicSaveConfig);

#ifdef PZEM_ON
  if (atoi(JConf.pzem_enable) == 1) {
    subTopicPzemReset.setCallback(CallbackPzemReset);
    mqtt.subscribe(&subTopicPzemReset);
  }
#endif

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: MqttSubscribe load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}


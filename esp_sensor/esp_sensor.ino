#include "ESP8266WiFi.h"
#include "WiFiUdp.h"
#include "ESP8266WebServer.h"
extern "C" {
#include "user_interface.h"
}

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "json_config.h"
#include "ArduinoJson.h"
#include "Wire.h"

#include "user_config.h"

#include "SimpleTimer.h"
SimpleTimer timer;

#if defined(NTP_ON)
#include "NTPClient.h"
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
#endif

#if defined(UART_ON)
#include "MY_ESP_UART.h"
Espuart Uart;
#endif

#if defined(DHT_ON)
#include <DHT.h>
// Uncomment the type of sensor in use:
//#define DHTTYPE           DHT11     // DHT 11
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
//#define DHTTYPE           DHT21     // DHT 21 (AM2301)
DHT dht(atoi(JConf.dht_pin), DHTTYPE);
#endif

#if defined(DS18X20_ON)
#include "OneWire.h"
OneWire ds(DS18X20_PIN);
#endif

#if defined(BH1750_ON)
#include "BH1750.h"
BH1750 lightSensor;
#endif

#if defined(BME280_ON)
#include "SparkFunBME280.h"
BME280 bmeSensor;
#endif

#if defined(SHT21_ON)
#include "HTU21D.h"
HTU21D myHTU21D;
#endif

#if defined(LCD_ON)
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2); // Устанавливаем дисплей
#endif

#if defined(PZEM_ON)
#include "PZEM004T.h"
PZEM004T pzem(&Serial);
IPAddress ip_pzem(192, 168, 1, 1);
float coil_ratio = 1.84; // Если используем разные катушки, подбираем коэффициент
enum PZEM_ENUM {PZEM_VOLTAGE, PZEM_CURRENT, PZEM_POWER, PZEM_ENERGY};
PZEM_ENUM pzem_current_read = PZEM_VOLTAGE;
enum PZEM_RESET_ENUM {PZEM_STAGE1, PZEM_STAGE2, PZEM_STAGE3, PZEM_STAGE4};
PZEM_RESET_ENUM pzem_reset_stage = PZEM_STAGE1;
#endif

WiFiUDP portUDP;            // UDP Syslog

ADC_MODE(ADC_VCC);
float voltage_float;

String network_html;        // Список доступных Wi-Fi точек

ESP8266WebServer WebServer(80);

extern WORKTIME_T worktime[2];

bool MqttConnect();

String AUTO;
String ON;
String OFF;
////////////////////////////////////////////////////////////////////////////////


bool check_worktime(WORKTIME_T wtime) {
  char log[LOGSZ];

#ifdef NTP_ON
  if (atoi(JConf.ntp_enable) == 1 && timeClient.update()) {
    unsigned long rawTime = timeClient.getEpochTime();
    unsigned int midnight_minutes = (rawTime % 86400L) / 60;
    snprintf_P(log, sizeof(log), PSTR("  Midnight Minutes: %d"), midnight_minutes);
    addLog(LOG_LEVEL_DEBUG_MORE, log);

    snprintf_P(log, sizeof(log), PSTR("  START Minutes: %d"), wtime.start_midn_minutes);
    addLog(LOG_LEVEL_DEBUG_MORE, log);
    snprintf_P(log, sizeof(log), PSTR("  STOP Minutes: %d"), wtime.stop_midn_minutes);
    addLog(LOG_LEVEL_DEBUG_MORE, log);

    if ((wtime.start_midn_minutes < wtime.stop_midn_minutes && (midnight_minutes < wtime.start_midn_minutes || midnight_minutes >= wtime.stop_midn_minutes))     // case A interv 1 3
        || (wtime.start_midn_minutes > wtime.stop_midn_minutes && midnight_minutes < wtime.start_midn_minutes && midnight_minutes >= wtime.stop_midn_minutes)) {  // case B interv 2
      addLog_P(LOG_LEVEL_DEBUG_MORE, "  time to light off");
      return false;
    }
  }
#endif
  addLog_P(LOG_LEVEL_DEBUG_MORE, "  time to light on");
  return true;
}

void Light1Control() {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: LightControl Start");

  if (light1State == ON) {
    worktime[0].state = HIGH;
  } else if (light1State == OFF) {
    worktime[0].state = LOW;
  } else if (light1State == AUTO) {
    addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: Light1");
    if (check_worktime(worktime[0]) && (atoi(JConf.bh1750_enable) == 0 || (atoi(JConf.bh1750_enable) == 1 && luxString.toInt() < atoi(JConf.light1_on_lux)))) {
      worktime[0].state = HIGH;
      addLog_P(LOG_LEVEL_DEBUG_MORE, "  > light1 on");
    } else {
      addLog_P(LOG_LEVEL_DEBUG_MORE, "  > light1 off");
      worktime[0].state = LOW;
    }
  }
  snprintf_P(log, sizeof(log), PSTR("Light 1 state: '%s' go to %s"), worktime[0].state ? "ON" : "OFF");
  addLog(LOG_LEVEL_INFO, log);
  digitalWrite(atoi(JConf.light1_pin), worktime[0].state);

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: LightControl load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}


void Light2Control() {
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: LightControl2 Start");

  if (light2State == ON) {
    worktime[1].state = HIGH;
  } else if (light2State == OFF) {
    worktime[1].state = LOW;
  } else if (light2State == AUTO) {
    addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: Light2");
    if (check_worktime(worktime[1]) && (atoi(JConf.bh1750_enable) == 0 || (atoi(JConf.bh1750_enable) == 1 && luxString.toInt() < atoi(JConf.light1_on_lux)))) {
      addLog_P(LOG_LEVEL_DEBUG_MORE, "  > light2 on");
      worktime[1].state = HIGH;
    } else {
      addLog_P(LOG_LEVEL_DEBUG_MORE, "  > light2 off");
      worktime[1].state = LOW;
    }
  }
  snprintf_P(log, sizeof(log), PSTR("Light 2 state: '%s' go to %s"), worktime[1].state ? "ON" : "OFF");
  addLog(LOG_LEVEL_INFO, log);

  digitalWrite(atoi(JConf.light2_pin), worktime[1].state);

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: LightControl2 load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}


void MotionDetect() {

  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: MotionDetect Start");

  if (digitalRead(atoi(JConf.motion_pin)) == HIGH) {
    addLog_P(LOG_LEVEL_INFO, "MotionDetect: movement detected");
    motionDetect = true;
    Light1Control();
    Light2Control();
    if (atoi(JConf.mqtt_enable) == 1 && mqtt.connected()) {
      pubTopicMotionSensor.publish("1");
    }
  } else if (motionDetect) {
    motionDetect = false;
    if (atoi(JConf.mqtt_enable) == 1 && mqtt.connected()) {
      pubTopicMotionSensor.publish("0");
    }
  }

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: MotionDetect load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}

String GetUptimeData() {

  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: GetUptimeData Start");

  //** Making Note of an expected rollover *****//
  if (millis() >= 3000000000) {
    HighMillis = 1;
  }
  //** Making note of actual rollover **//
  if (millis() <= 100000 && HighMillis == 1) {
    Rollover++;
    HighMillis = 0;
  }

  long secsUp = millis() / 1000;
  Second = secsUp % 60;
  Minute = (secsUp / 60) % 60;
  Hour = (secsUp / (60 * 60)) % 24;
  Day = (Rollover * 50) + (secsUp / (60 * 60 * 24)); //First portion takes care of a rollover [around 50 days]

  sprintf_P(value_buff, (const char *)F("%dd %02d:%02d"), Day, Hour, Minute);
  uptimeString = String(value_buff);

  snprintf_P(log, sizeof(log), PSTR("GetUptimeData: Uptime: %s:%d%d"), uptimeString.c_str(), Second / 10, Second % 10);
  addLog(LOG_LEVEL_DEBUG, log);

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: GetUptimeData load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);

  return value_buff;
}

#ifdef NTP_ON
void NTPSettingsUpdate() {
  if (atoi(JConf.ntp_enable) == 1) {
    ntpLastUpdateTime = millis();
    timeClient.end();
    timeClient.setUpdateServer(JConf.ntp_server);
    timeClient.setTimeOffset(atoi(JConf.my_time_zone) * 60 * 60);
    timeClient.setUpdateInterval(60 * 60 * 1000);
    timeClient.begin();
  }
}
#endif


unsigned int worktime_tominute(char* str) {
  char hours[3] = {str[0], str[1]};
  char minutes[3] = {str[3], str[4]};
  return atoi(hours) * 60 + atoi(minutes);
}

void WorkTimeSettingsUpdate() {
  worktime[0].start_midn_minutes = worktime_tominute(JConf.light1_start_time);
  worktime[0].stop_midn_minutes = worktime_tominute(JConf.light1_stop_time);
  worktime[1].start_midn_minutes = worktime_tominute(JConf.light2_start_time);
  worktime[1].stop_midn_minutes = worktime_tominute(JConf.light2_stop_time);
}

void TestSystemPrint()
{
  char log[LOGSZ];
  unsigned long start_time = millis();
  addLog_P(LOG_LEVEL_DEBUG_MORE, "Func: TestSystemPrint Start");

  snprintf_P(log, sizeof(log), PSTR("ESP: Version: %s"), ver);
  addLog(LOG_LEVEL_DEBUG, log);

  snprintf_P(log, sizeof(log), PSTR("ESP: IP address: %s"), ipString.c_str());
  addLog(LOG_LEVEL_DEBUG, log);

  snprintf_P(log, sizeof(log), PSTR("ESP: Sketch size: %d"), ESP.getSketchSize());
  addLog(LOG_LEVEL_DEBUG, log);

  snprintf_P(log, sizeof(log), PSTR("ESP: Free size: %d"), ESP.getFreeSketchSpace());
  addLog(LOG_LEVEL_DEBUG, log);

  snprintf_P(log, sizeof(log), PSTR("ESP: Free memory: %s"), freeMemoryString.c_str());
  addLog(LOG_LEVEL_DEBUG, log);

  snprintf_P(log, sizeof(log), PSTR("ESP: Flash Chip Size: %d"), ESP.getFlashChipSize());
  addLog(LOG_LEVEL_DEBUG_MORE, log);

  snprintf_P(log, sizeof(log), PSTR("ESP: Flash Chip Speed: %d"), ESP.getFlashChipSpeed());
  addLog(LOG_LEVEL_DEBUG_MORE, log);

  unsigned long load_time = millis() - start_time;
  snprintf_P(log, sizeof(log), PSTR("Func: TestSystemPrint load time: %d"), load_time);
  addLog(LOG_LEVEL_DEBUG_MORE, log);
}


void getData() {
#ifdef NTP_ON
  if (atoi(JConf.ntp_enable) == 1) {
    ntpTimeString = timeClient.getFormattedTime();
  }
#endif

  int voltage = ESP.getVcc();
  voltage_float = voltage / 1000.0;

#ifdef BH1750_ON
  if (atoi(JConf.bh1750_enable) == 1) {
    GetLightSensorData();
  }
#endif

#ifdef BME280_ON
  if (atoi(JConf.bme280_enable) == 1) {
    GetBmeSensorData();
  }
#endif

#ifdef SHT21_ON
  if (atoi(JConf.sht21_enable) == 1) {
    GetSHT21SensorData();
  }
#endif

#ifdef DHT_ON
  if (atoi(JConf.dht_enable) == 1) {
    GetDhtSensorData();
  }
#endif

#ifdef DS18X20_ON
  if (atoi(JConf.ds18x20_enable) == 1) {
    if (searchDsSensorDone) {
      GetDS18x20SensorData();
    } else {
      SearchDS18x20Sensors();
    }
  }
#endif //DS18X20_ON

#ifdef PZEM_ON
  if (atoi(JConf.pzem_enable) == 1) {
    GetPzemSerialRead();
  }
#endif

#ifdef MHZ19_ON
  if (atoi(JConf.mhz19_enable) == 1) {
    GetMHZ19();
  }
#endif

  GetUptimeData();
  GetFreeMemory();
  TestSystemPrint();

#ifdef LCD_ON
  //scanI2C();
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Test");
#endif

#ifdef UART_ON
  for (int i = 0; i < ANALOG_PINS; i++) {
    if (millis() - Uart.timerAnalogPin[i] >= 60000) {
      Uart.valueAnalogPin[i] = 0;
      Uart.SetAnalogReadCycle(i, 10, "s");
    }
  }
#endif
}

#ifdef LCD_ON
void scanI2C() {
  char log[LOGSZ];
  byte error, address;
  int nDevices;
  nDevices = 0;

  for (address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      snprintf_P(log, sizeof(log), PSTR("I2C device found at address 0x%x !"), address);
      addLog(LOG_LEVEL_INFO, log);
      nDevices++;
    } else if (error == 4) {
      snprintf_P(log, sizeof(log), PSTR("Unknown error at address 0x%x !"), address);
      addLog(LOG_LEVEL_ERROR, log);
    }
  }
  if (nDevices == 0) {
    addLog_P(LOG_LEVEL_ERROR, "No I2C devices found");
  }
}
#endif //LCD_ON

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();

  if (!SPIFFS.begin()) {
    addLog_P(LOG_LEVEL_NONE, "setup: Failed to mount file system");
    return;
  } else {
#ifdef RESET_BUTTON_ON
    deleteConfigFile();
#endif
  }

  //JConf.deleteConfig();

  if (!JConf.loadConfig()) {
    addLog_P(LOG_LEVEL_NONE, "setup: Failed to load config");
  } else {
    addLog_P(LOG_LEVEL_NONE, "setup: Config loaded");
  }

  AUTO += FPSTR(AUTOP);
  ON += FPSTR(ONP);
  OFF += FPSTR(OFFP);

  if (atoi(JConf.pzem_enable) == 1 || atoi(JConf.mhz19_enable) == 1) {
    JConf.serial_log_level[0] = '0'; // Отключаем serial log
    JConf.serial_log_level[1] = '\0';
    Serial.end();
    Serial.begin(9600);
    delay(100);
    Serial.println();
  } else {
    JConf.printConfig();
  }

  pinMode(atoi(JConf.light1_pin), OUTPUT);
  pinMode(atoi(JConf.light2_pin), OUTPUT);
  pinMode(atoi(JConf.motion_pin), INPUT);

  digitalWrite(atoi(JConf.light1_pin), LOW);
  digitalWrite(atoi(JConf.light2_pin), LOW);

  light1State = AUTO;
  light2State = AUTO;
  WorkTimeSettingsUpdate();

  if (!WiFiSetup()) {
    WiFiSafeSetup();
  }
  delay(300);

#ifdef PZEM_ON
  pzem.setAddress(ip_pzem);
  pzem.setReadTimeout(500);
#endif

#ifdef DHT_ON
  dht = DHT(atoi(JConf.dht_pin), DHTTYPE);
  dht.begin();
#endif

#ifdef BME280_ON
  if (atoi(JConf.bme280_enable) == 1) {
    bmeSensor.settings.commInterface = I2C_MODE;
    bmeSensor.settings.I2CAddress = 0x76;
    bmeSensor.settings.runMode = 3;
    bmeSensor.settings.tStandby = 0;
    bmeSensor.settings.filter = 4;
    bmeSensor.settings.tempOverSample = 5;
    bmeSensor.settings.pressOverSample = 5;
    bmeSensor.settings.humidOverSample = 5;
    bmeSensor.begin();
  }
#endif

#ifdef BH1750_ON
  if (atoi(JConf.bh1750_enable) == 1) {
    lightSensor.begin();
  }
#endif

  if (atoi(JConf.bme280_enable) == 1 || atoi(JConf.bh1750_enable) == 1 || atoi(JConf.sht21_enable) == 1) {
    Wire.setClock(100000);
  }

#ifdef SHT21_ON
  myHTU21D.begin(4, 5);  //SDA=4, SCL=5
#endif

#ifdef LCD_ON
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 1);
  lcd.print("Test");
#endif //LCD_ON

#ifdef ENCODER_ON
  encoderSetup();
#endif //ENCODER_ON

  if (atoi(JConf.mqtt_enable) == 1) {
    if (atoi(JConf.mqtt_auth_enable) == 1) {
      mqtt = Adafruit_MQTT_Client(&espClient, JConf.mqtt_server, atoi(JConf.mqtt_port), JConf.mqtt_user, JConf.mqtt_pwd);
    } else {
      mqtt = Adafruit_MQTT_Client(&espClient, JConf.mqtt_server, atoi(JConf.mqtt_port));
    }
    MqttInit();
    MqttSubscribe();
    MqttConnect();
  }

#ifdef USE_WEBSERVER
  WebServerInit();
#endif // USE_WEBSERVER

#ifdef NTP_ON
  if (atoi(JConf.ntp_enable) == 1) {
    NTPSettingsUpdate();
    ntpTimer = timer.setInterval(NTP_TIME_SLEEP, NTPSettingsUpdate);
  }
#endif

  wifiReconnectTimer = timer.setInterval(10000, wifiReconnect);
  timer.setInterval(atoi(JConf.get_data_delay) * 1000, getData);

  timer.setInterval(60000, MqttConnect);
  timer.setInterval(atoi(JConf.publish_delay) * 1000, MqttPubData);

  subscribeTimer = timer.setInterval(atoi(JConf.subscribe_delay) * 1000, MqttSubscribe);
  timer.setInterval(600000, wifiSafeModeReconnect);

#ifdef REBOOT_ON
  rebootTimer = timer.setInterval(atoi(JConf.reboot_delay) * 1000, restartESP);
#endif

  GetMacString();
}


void loop() {
  static unsigned long previousMillis;
  if (millis() - previousMillis >= 1000) {
    previousMillis += 1000;
    if (light1State == AUTO) Light1Control();
    if (light2State == AUTO) Light2Control();
  }

#ifdef USE_WEBSERVER
  WebServer.handleClient();  // handle web server
#endif // USE_WEBSERVER

  timer.run();

  if (WiFi.status() == WL_CONNECTED && atoi(JConf.mqtt_enable) == 1) {
    if (mqtt.connected()) {
      mqtt.processPackets(100);
    }
  }

  if (atoi(JConf.motion_sensor_enable) == 1 && motionDetect == false) {
    MotionDetect();
  }

#ifdef UART_ON
  Uart.serialEvent();
#endif


#ifdef NTP_ON
  if ( atoi(JConf.ntp_enable) == 1 && millis() - ntpLastUpdateTime < NTP_ERROR_TIME ) {
    if (timeClient.update()) ntpLastUpdateTime = millis();
    timer.restartTimer(ntpTimer);
  }
#endif

  yield();
}

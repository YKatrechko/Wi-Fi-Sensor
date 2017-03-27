#ifndef JSON_CONFIG_H
#define JSON_CONFIG_H

#include "Arduino.h"
#include "ArduinoJson.h"
#include "FS.h"

const String ConfigFileName = "/conf.json";
#define DEBUG_JSON_CONFIG

class JsonConf
{
  public:
    bool saveConfig();
    bool loadConfig();
    void deleteConfig();
    bool printConfig();

    char module_id                    [32] = "ESP8266"            ;
    char wifi_mode                    [7]  = "STA"                ;
    char wifi_phy_mode                [4]  = "11G"                ;
    char wifi_channel                 [3]  = "12"                 ;
    char wifi_auth                    [13]  = "WPA2_PSK"          ;
    char sta_ssid                     [32] = "AAA0"               ;
    char sta_pwd                      [32] = "WPA7305023"         ;
    char ap_pwd                       [32] = "Poiu0987"           ;
    char static_ip                    [16] = "192.168.2.220"      ;
    char static_gateway               [16] = "192.168.2.1"        ;
    char static_subnet                [16] = "255.255.255.0"      ;
    char ntp_server                   [32] = "europe.pool.ntp.org";
    char my_time_zone                 [4]  = "+3"                 ;
    char mqtt_server                  [32] = "192.168.2.205"      ;
    char mqtt_port                    [6]  = "1883"               ;
    char mqtt_user                    [32] = "none"               ;
    char mqtt_pwd                     [32] = "none"               ;
    char mqtt_name                    [32] = "_BedM"              ;
    char publish_topic                [32] = "/stateSub/"         ;
    char subscribe_topic              [32] = "/statePub/"         ;
    char command_pub_topic            [32] = "/commandPub/"       ;
    char light1_pin                   [3]  = "13"                 ;
    char light1_off_delay             [32] = "5"                  ;
    char light1_on_lux                [6]  = "10"                 ; // Значение в люксах, ниже которого будет включаться нагрузка
    char light1_smooth                [2]  = "0"                  ;
    char light2_pin                   [3]  = "12"                 ;
    char light2_off_delay             [32] = "5"                  ;
    char light2_on_lux                [6]  = "10"                 ; // Значение в люксах, ниже которого будет включаться нагрузка
    char light2_smooth                [2]  = "0"                  ;
    char reset_pin                    [3]  = "2"                  ;
    char motion_pin                   [3]  = "14"                 ;
    char dht_pin                      [3]  = "2"                  ;
    char encoder_pin_a                [3]  = "4"                  ;
    char encoder_pin_b                [3]  = "5"                  ;
    char get_data_delay               [32] = "10"                 ;
    char publish_delay                [32] = "10"                 ;
    char subscribe_delay              [32] = "60"                 ;
    char motion_read_delay            [32] = "10"                 ;
    char reboot_delay                 [32] = "1800"               ;

    char sys_log_host                 [33] = "192.168.2.50"       ;
    char sys_log_port                 [6]  = "514"                ;
    char sys_log_level                [2]  = "0"                  ;
    char serial_log_level             [2]  = "3"                  ;
    char web_log_level                [2]  = "2"                  ;

    char static_ip_enable             [2]  = "0"                  ;
    char ntp_enable                   [2]  = "1"                  ;
    char mqtt_enable                  [2]  = "0"                  ;
    char mqtt_auth_enable             [2]  = "0"                  ;
    char dht_enable                   [2]  = "0"                  ;
    char bme280_enable                [2]  = "0"                  ;
    char sht21_enable                 [2]  = "0"                  ;
    char bh1750_enable                [2]  = "0"                  ;
    char motion_sensor_enable         [2]  = "0"                  ;
    char pzem_enable                  [2]  = "0"                  ;
    char mhz19_enable                 [2]  = "0"                  ;
    char ds18x20_enable               [2]  = "0"                  ;
    char light1_start_time            [6]  = "00:00"              ;
    char light1_stop_time             [6]  = "24:00"              ;
    char light2_start_time            [6]  = "00:00"              ;
    char light2_stop_time             [6]  = "24:00"              ;

  private:
};

#endif

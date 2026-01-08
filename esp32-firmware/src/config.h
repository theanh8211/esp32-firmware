#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <HTTPUpdate.h>
#include <esp_task_wdt.h>
#include <DHT.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <LittleFS.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define SDA_PIN 4
#define SCL_PIN 5

#define DHT_PIN 19
#define DHT_TYPE DHT11

#define SOIL1_PIN 34
#define SOIL2_PIN 35
#define LDR_PIN 32
#define PH_PIN 39

#define RELAY_PUMP 18
#define RELAY_FAN 21
#define RELAY_LIGHT 22

#define BTN_OK 25
#define BTN_BACK 14
#define BTN_LEFT 27
#define BTN_RIGHT 33
#define BTN_UP 26
#define BTN_DOWN 13

#define EEPROM_SIZE 512
#define FIRMWARE_VERSION "1.2.3"
#define SERVER_IP "192.168.31.44"

#define SERVER_PORT 80
#define MAX_SCHEDULES 10
#define MAX_WIFI_NETWORKS 10

extern const char* DEFAULT_SSID;
extern const char* DEFAULT_PASS;

void feedWatchdog();

extern hd44780_I2Cexp lcd;
extern uint8_t dhtFailCount;
extern DHT dht;
extern WiFiUDP ntpUDP;
extern NTPClient timeClient;
extern bool ntpSynced;
extern bool lcdAvailable;
// Mutex to protect LCD I2C access from multiple tasks
extern SemaphoreHandle_t lcdMutex;
// RTOS task handles (for health/debug)
extern TaskHandle_t serverTaskHandle;
extern TaskHandle_t buttonTaskHandle;
extern TaskHandle_t lcdTaskHandle;
extern TaskHandle_t sensorTaskHandle;
// Request main-screen refresh when background tasks update data
extern bool needMainRefresh;
// suppress applying remote updates for short window after local edits
extern unsigned long suppressRemoteUntil;

struct Schedule {
  uint8_t hour;
  uint8_t minute;
  bool forPump;
  bool forLight;
};

struct Settings {
  char ssid[32];
  char pass[64];
  float tempThresh;
  float humThresh;
  float soilThresh;
  float lightThresh;
  float phThreshMin;
  float phThreshMax;
  bool dailyWater;
  bool lightAuto;
  bool pumpAuto;
  bool fanAuto;
  bool deepSleep;
  bool relayOverride;
  char deviceID[16];
  char token[32];
  bool addedToWeb;
  Schedule schedules[MAX_SCHEDULES];
  uint8_t numSchedules;
  // MQTT settings persisted to EEPROM so devices can be configured remotely
  char mqttBroker[64];
  uint16_t mqttPort;
  char mqttUser[32];
  char mqttPass[64];
  bool mqttUseTLS;
};
extern Settings settings;

enum MenuState { MAIN_SCREEN, MAIN_MENU, SCHEDULE_MENU, EDIT_SCHEDULE, THRESHOLD_MENU, EDIT_TEMP, EDIT_HUM, EDIT_SOIL, EDIT_LIGHT, EDIT_PH_MIN, EDIT_PH_MAX, LIGHT_SET_MENU, VERSION_MENU, INFO_MENU, MANUAL_CONTROL, WIFI_SETUP, AUTO_CONTROL_MENU };
extern MenuState menuState;

extern int menuIndex, subIndex, editIndex;
extern Schedule tempSchedule;
extern char inputBuffer[64];
extern int inputPos;

#define MQTT_BROKER "192.168.31.44"
#define MQTT_PORT 1883
#define MQTT_USER "smartfarm"
#define MQTT_PASS "xsdHIKRFxqRslaxtqSsYeQ"

// Heartbeat / status settings
// How often the device publishes a heartbeat (seconds)
#define HEARTBEAT_INTERVAL_SECONDS 30
// Topic formats (use snprintf or String formatting in code)
#define STATUS_TOPIC_FMT "devices/%s/status"
#define HEARTBEAT_TOPIC_FMT "devices/%s/heartbeat"

// MQTT API (implemented in mqtt_client.cpp)
void mqtt_init();
void mqtt_loop();
void mqtt_publishTelemetry();
void mqtt_publishHeartbeat();

#endif
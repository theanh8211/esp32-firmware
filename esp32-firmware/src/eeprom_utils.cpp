// eeprom_utils.cpp
#include "eeprom_utils.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

Settings settings;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7*3600, 60000);
bool ntpSynced = false;

// LCD mutex (defined once)
SemaphoreHandle_t lcdMutex = NULL;
// Task handles
TaskHandle_t serverTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;
TaskHandle_t lcdTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;

// When set by background tasks, UI will refresh main screen at next opportunity
bool needMainRefresh = false;
// When set to millis() + delta, remote incoming config updates for thresholds
// will be ignored until this time to avoid overwriting user in-progress edits.
unsigned long suppressRemoteUntil = 0;

// Gán giá trị default WiFi
const char* DEFAULT_SSID = "Ngaos";
const char* DEFAULT_PASS = "Gio1108@";

// EEPROM debounce / safety
static volatile bool eepromPending = false;
static volatile unsigned long eepromPendingSince = 0;
static unsigned long lastEepromWrite = 0;
unsigned long eepromWriteCount = 0;
static const unsigned long EEPROM_COMMIT_DEBOUNCE_MS = 5000; // 5s debounce

// CRC32 helper
static uint32_t crc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  while (len--) {
    uint8_t byte = *data++;
    crc ^= byte;
    for (uint8_t i = 0; i < 8; ++i) crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
  }
  return ~crc;
}

void initEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  // start eeprom writer task
  xTaskCreatePinnedToCore([](void*){
    const TickType_t delay = pdMS_TO_TICKS(500);
    while (1) {
      if (eepromPending) {
        if (millis() - eepromPendingSince >= EEPROM_COMMIT_DEBOUNCE_MS) {
          // perform immediate write
          // compute crc and write
          uint32_t crc = crc32((uint8_t*)&settings, sizeof(Settings));
          EEPROM.put(0, crc);
          EEPROM.put(4, settings);
          EEPROM.commit();
          lastEepromWrite = millis();
          eepromWriteCount++;
          eepromPending = false;
        }
      }
      vTaskDelay(delay);
    }
  }, "EEPROMTask", 4096, NULL, 1, NULL, 1);
}

void loadSettings() {
  // Read CRC and settings; validate CRC before using settings
  uint32_t storedCrc = 0;
  EEPROM.get(0, storedCrc);
  EEPROM.get(4, settings);
  // ensure string termination
  settings.ssid[31] = '\0';
  settings.pass[63] = '\0';

  uint32_t calc = crc32((uint8_t*)&settings, sizeof(Settings));
  if (storedCrc != calc) {
    // EEPROM invalid — try to recover device identity from LittleFS first
    memset(&settings, 0x00, sizeof(Settings));
    // attempt to mount LittleFS and read identity file
    bool gotIdentity = false;
    if (LittleFS.begin()) {
      if (LittleFS.exists("/identity.json")) {
        File f = LittleFS.open("/identity.json", "r");
        if (f) {
          DynamicJsonDocument iddoc(256);
          DeserializationError derr = deserializeJson(iddoc, f);
          if (!derr) {
            const char* did = iddoc["deviceID"];
            const char* tok = iddoc["token"];
            if (did && tok) {
              strncpy(settings.deviceID, did, sizeof(settings.deviceID)-1);
              settings.deviceID[sizeof(settings.deviceID)-1] = '\0';
              strncpy(settings.token, tok, sizeof(settings.token)-1);
              settings.token[sizeof(settings.token)-1] = '\0';
              gotIdentity = true;
            }
          }
          f.close();
        }
      }
      // avoid formatting LittleFS here — only read identity
    }

    // If we recovered identity from LittleFS but EEPROM CRC was invalid,
    // ensure other important fields have sensible defaults and persist
    // the recovered identity back to EEPROM so future boots are stable.
    if (gotIdentity) {
      // set sane defaults only when values look unset (0)
      if (settings.tempThresh == 0.0f) settings.tempThresh = 28.0;
      if (settings.humThresh == 0.0f) settings.humThresh = 60.0;
      if (settings.soilThresh == 0.0f) settings.soilThresh = 50.0;
      if (settings.lightThresh == 0.0f) settings.lightThresh = 500.0;
      if (settings.phThreshMin == 0.0f) settings.phThreshMin = 5.5;
      if (settings.phThreshMax == 0.0f) settings.phThreshMax = 7.5;
      if (settings.numSchedules == 0) {
        settings.numSchedules = 1;
        settings.schedules[0] = {2, 50, true, false};
      }
      // Persist immediately so CRC and EEPROM are consistent
      saveSettingsNow();
    }

    if (!gotIdentity) {
      // no persisted identity available -> will generate new one below
    }
  }

  if (settings.deviceID[0] == 0xFF || settings.deviceID[0] == '\0') {
    randomSeed(millis());
    // Generate compact, human-friendly ID/token on each flash
    // ID: ESPxxxx (4 digits), token: 4-digit numeric string
    int idNum = random(0, 10000);
    int tokNum = random(0, 10000);
    snprintf(settings.deviceID, 16, "ESP%04d", idNum);
    snprintf(settings.token, 32, "%04d", tokNum);
    settings.tempThresh = 28.0;
    settings.humThresh = 60.0;
    settings.soilThresh = 50.0;
    settings.lightThresh = 500.0;
    settings.phThreshMin = 5.5;
    settings.phThreshMax = 7.5;
    settings.dailyWater = true;
    settings.lightAuto = true;
    settings.pumpAuto = true;
    settings.fanAuto = true;
    settings.deepSleep = false;
    settings.relayOverride = false;
    settings.addedToWeb = false;
    settings.numSchedules = 1;
    settings.schedules[0] = {2, 50, true, false};
    // default MQTT values (compile-time fallbacks from config.h macros)
    strncpy(settings.mqttBroker, MQTT_BROKER, sizeof(settings.mqttBroker)-1);
    settings.mqttBroker[sizeof(settings.mqttBroker)-1] = '\0';
    settings.mqttPort = MQTT_PORT;
    strncpy(settings.mqttUser, MQTT_USER, sizeof(settings.mqttUser)-1);
    settings.mqttUser[sizeof(settings.mqttUser)-1] = '\0';
    strncpy(settings.mqttPass, "", sizeof(settings.mqttPass)-1);
    settings.mqttPass[sizeof(settings.mqttPass)-1] = '\0';
    settings.mqttUseTLS = false;
    saveSettings();
    // persist identity to LittleFS so future flashes can recover stable ID/token
    if (LittleFS.begin()) {
      DynamicJsonDocument iddoc(256);
      iddoc["deviceID"] = String(settings.deviceID);
      iddoc["token"] = String(settings.token);
      File f = LittleFS.open("/identity.json", "w");
      if (f) {
        serializeJson(iddoc, f);
        f.close();
      }
    }
  }

  // Sửa: Nếu SSID rỗng hoặc không hợp lệ → dùng mặc định
  bool wifiInvalid = false;

  // check byte đầu tiên là 0xFF (EEPROM rỗng)
  if ((uint8_t)settings.ssid[0] == 0xFF) wifiInvalid = true;
  if ((uint8_t)settings.pass[0] == 0xFF) wifiInvalid = true;

  // check độ dài hợp lý
  if (strlen(settings.ssid) == 0 || strlen(settings.ssid) > 31) wifiInvalid = true;
  if (strlen(settings.pass) < 8 || strlen(settings.pass) > 63) wifiInvalid = true;

  if (wifiInvalid) {
    Serial.println("EEPROM WiFi invalid (0xFF or length), using default WiFi");

    memset(settings.ssid, 0, sizeof(settings.ssid));
    memset(settings.pass, 0, sizeof(settings.pass));

    strncpy(settings.ssid, DEFAULT_SSID, sizeof(settings.ssid) - 1);
    strncpy(settings.pass, DEFAULT_PASS, sizeof(settings.pass) - 1);

    saveSettings();
  }

  // Ensure mqtt strings are NUL-terminated and valid after load
  settings.mqttBroker[sizeof(settings.mqttBroker)-1] = '\0';
  settings.mqttUser[sizeof(settings.mqttUser)-1] = '\0';
  settings.mqttPass[sizeof(settings.mqttPass)-1] = '\0';

}

void saveSettings() {
  // mark pending write and debounce in background task
  eepromPending = true;
  eepromPendingSince = millis();
}

void saveSettingsNow() {
  // immediate write to EEPROM (compute CRC and commit)
  uint32_t crc = crc32((uint8_t*)&settings, sizeof(Settings));
  EEPROM.put(0, crc);
  EEPROM.put(4, settings);
  EEPROM.commit();
  lastEepromWrite = millis();
  eepromWriteCount++;
  eepromPending = false;
  // Also persist identity to LittleFS (best-effort, does not format)
  if (LittleFS.begin()) {
    DynamicJsonDocument iddoc(256);
    iddoc["deviceID"] = String(settings.deviceID);
    iddoc["token"] = String(settings.token);
    File f = LittleFS.open("/identity.json", "w");
    if (f) {
      serializeJson(iddoc, f);
      f.close();
    }
  }
}

void clearEEPROM() {
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0xFF);
  }
  EEPROM.commit();
  Serial.println("EEPROM cleared - will use default WiFi on restart");
}
#include "mqtt_client.h"
#include "config.h"
#include "eeprom_utils.h"
#include "relay_control.h"
#include "lcd_menu.h"
#include "sensors.h"
#include "wifi_server.h"
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

// Use plain WiFiClient for non-TLS; WiFiClientSecure if TLS needed
static WiFiClient wifiClient;
static WiFiClientSecure wifiClientSecure;
static PubSubClient mqttClient(wifiClient);

static String topicConfig;
static String topicTelemetry;
static String topicHeartbeat;

void applyConfigFromJson(JsonObjectConst obj) {
  // If user is actively editing thresholds on-device, avoid applying remote
  // threshold updates for a short suppression window to prevent overwriting
  // in-progress edits.
  bool suppress = (millis() < suppressRemoteUntil) || (menuState >= EDIT_TEMP && menuState <= EDIT_PH_MAX);
  if (obj.containsKey("tempThresh") && !suppress) settings.tempThresh = obj["tempThresh"].as<float>();
  if (obj.containsKey("temperature_threshold") && !suppress) settings.tempThresh = obj["temperature_threshold"].as<float>();
  if (obj.containsKey("humThresh") && !suppress) settings.humThresh = obj["humThresh"].as<float>();
  if (obj.containsKey("humidity_threshold") && !suppress) settings.humThresh = obj["humidity_threshold"].as<float>();
  if (obj.containsKey("soilThresh") && !suppress) settings.soilThresh = obj["soilThresh"].as<float>();
  if (obj.containsKey("soil_threshold") && !suppress) settings.soilThresh = obj["soil_threshold"].as<float>();
  if (obj.containsKey("lightThresh") && !suppress) settings.lightThresh = obj["lightThresh"].as<float>();
  if (obj.containsKey("light_threshold") && !suppress) settings.lightThresh = obj["light_threshold"].as<float>();
  if (obj.containsKey("phThreshMin") && !suppress) settings.phThreshMin = obj["phThreshMin"].as<float>();
  if (obj.containsKey("phThreshMax") && !suppress) settings.phThreshMax = obj["phThreshMax"].as<float>();
  if (obj.containsKey("dailyWater")) settings.dailyWater = obj["dailyWater"].as<bool>();
  if (obj.containsKey("lightAuto") && !suppress) settings.lightAuto = obj["lightAuto"].as<bool>();
  if (obj.containsKey("light_auto") && !suppress) settings.lightAuto = obj["light_auto"].as<bool>();
  if (obj.containsKey("pumpAuto") && !suppress) { settings.pumpAuto = obj["pumpAuto"].as<bool>(); if (settings.pumpAuto) settings.relayOverride = false; }
  if (obj.containsKey("pump_auto") && !suppress) { settings.pumpAuto = obj["pump_auto"].as<bool>(); if (settings.pumpAuto) settings.relayOverride = false; }
  if (obj.containsKey("fanAuto") && !suppress) { settings.fanAuto = obj["fanAuto"].as<bool>(); if (settings.fanAuto) settings.relayOverride = false; }
  if (obj.containsKey("fan_auto") && !suppress) { settings.fanAuto = obj["fan_auto"].as<bool>(); if (settings.fanAuto) settings.relayOverride = false; }
  if (obj.containsKey("addedToWeb")) settings.addedToWeb = obj["addedToWeb"].as<bool>();
  // If relay_override requested, honor it only when the requested auto flags do not
  // indicate autos should remain active. This ensures a single MQTT payload that
  // disables autos and requests relay_override is applied correctly.
  if (obj.containsKey("relay_override")) {
    bool ro = obj["relay_override"].as<bool>();
    bool hasPumpAuto = obj.containsKey("pumpAuto") || obj.containsKey("pump_auto");
    bool hasFanAuto = obj.containsKey("fanAuto") || obj.containsKey("fan_auto");
    bool hasLightAuto = obj.containsKey("lightAuto") || obj.containsKey("light_auto");
    bool requestedPump = hasPumpAuto ? (obj.containsKey("pumpAuto") ? obj["pumpAuto"].as<bool>() : obj["pump_auto"].as<bool>()) : settings.pumpAuto;
    bool requestedFan = hasFanAuto ? (obj.containsKey("fanAuto") ? obj["fanAuto"].as<bool>() : obj["fan_auto"].as<bool>()) : settings.fanAuto;
    bool requestedLight = hasLightAuto ? (obj.containsKey("lightAuto") ? obj["lightAuto"].as<bool>() : obj["light_auto"].as<bool>()) : settings.lightAuto;
    bool requestedAutoActive = requestedPump || requestedFan || requestedLight;
    if (!(ro && requestedAutoActive)) {
      settings.relayOverride = ro;
    } else {
      Serial.println("[MQTT] Ignoring relay_override because requested auto flags remain active");
    }
  }
  if (obj.containsKey("schedules")) {
    if (!suppress) {
      JsonArrayConst arr = obj["schedules"].as<JsonArrayConst>();
      settings.numSchedules = min((uint8_t)arr.size(), (uint8_t)MAX_SCHEDULES);
      for (uint8_t i = 0; i < settings.numSchedules; i++) {
        settings.schedules[i].hour = arr[i]["hour"];
        settings.schedules[i].minute = arr[i]["minute"];
        settings.schedules[i].forPump = arr[i]["forPump"];
        settings.schedules[i].forLight = arr[i]["forLight"];
      }
    }
  }
  // persist and apply
  saveSettingsNow();
  // Apply manual relay commands from MQTT payload (mirror HTTP /apply_config behavior)
  if (obj.containsKey("pump")) {
    bool p = obj["pump"].as<bool>();
    if (!settings.pumpAuto) { state.pump = p; digitalWrite(RELAY_PUMP, state.pump); }
    else Serial.println("[MQTT] Ignoring remote pump command because pumpAuto is enabled");
  }
  if (obj.containsKey("fan")) {
    bool f = obj["fan"].as<bool>();
    if (!settings.fanAuto) { state.fan = f; digitalWrite(RELAY_FAN, state.fan); }
    else Serial.println("[MQTT] Ignoring remote fan command because fanAuto is enabled");
  }
  if (obj.containsKey("lightOn")) {
    bool l = obj["lightOn"].as<bool>();
    if (!settings.lightAuto) { state.lightOn = l; digitalWrite(RELAY_LIGHT, state.lightOn); }
    else Serial.println("[MQTT] Ignoring remote light command because lightAuto is enabled");
  }
  controlRelays();
  // Request UI refresh only if main screen active; otherwise mark for refresh
  if (menuState == MAIN_SCREEN) drawMainScreen(); else needMainRefresh = true;
  // notify backend immediately so web reflects new values
  requestTelemetrySend();
}

static void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // parse JSON payload
  StaticJsonDocument<2048> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.printf("[MQTT] invalid JSON: %s\n", err.c_str());
    return;
  }
  JsonObject obj = doc.as<JsonObject>();
  Serial.println("[MQTT] config message received");
  applyConfigFromJson(obj);
}

static bool mqttConnect() {
  if (mqttClient.connected()) return true;
  mqttClient.setCallback(mqttCallback);
  String clientId = String(settings.deviceID) + "-" + String((uint32_t)ESP.getEfuseMac());
  bool ok = false;
  const char* broker = settings.mqttBroker[0] ? settings.mqttBroker : MQTT_BROKER;
  uint16_t port = settings.mqttPort ? settings.mqttPort : MQTT_PORT;
  const char* user = settings.mqttUser[0] ? settings.mqttUser : MQTT_USER;
  const char* pass = settings.mqttPass[0] ? settings.mqttPass : MQTT_PASS;
  mqttClient.setServer(broker, port);
  if (user && strlen(user) > 0) {
    ok = mqttClient.connect(clientId.c_str(), user, pass, (String("devices/") + settings.deviceID + "/status").c_str(), 1, true, "offline");
  } else {
    ok = mqttClient.connect(clientId.c_str());
  }
    if (ok) {
    Serial.println("[MQTT] connected");
    topicConfig = String("devices/") + settings.deviceID + "/config";
    topicTelemetry = String("devices/") + settings.deviceID + "/telemetry";
    topicHeartbeat = String("devices/") + settings.deviceID + "/heartbeat";
    mqttClient.subscribe(topicConfig.c_str());
    // publish online status retained
    StaticJsonDocument<128> s;
    s["online"] = true;
    char buf[128]; size_t n = serializeJson(s, buf);
    mqttClient.publish((String("devices/") + settings.deviceID + "/status").c_str(), (const uint8_t*)buf, n, true);
    return true;
  } else {
    Serial.println("[MQTT] connect failed");
    return false;
  }
}

void mqtt_init() {
  // choose secure client based on runtime settings if provided
  const char* broker = settings.mqttBroker[0] ? settings.mqttBroker : MQTT_BROKER;
  uint16_t port = settings.mqttPort ? settings.mqttPort : MQTT_PORT;
  bool useTLS = settings.mqttUseTLS;
  if (useTLS || port == 8883) {
    // For testing you can set insecure; in production set CA with setCACert()
    wifiClientSecure.setInsecure();
    mqttClient.setClient(wifiClientSecure);
  } else {
    mqttClient.setClient(wifiClient);
  }
  mqttClient.setServer(broker, port);
  mqttConnect();
}

void mqtt_loop() {
  if (!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();
  // publish heartbeat periodically
  static unsigned long lastHeartbeatMs = 0;
  unsigned long now = millis();
  if (now - lastHeartbeatMs >= (unsigned long)HEARTBEAT_INTERVAL_SECONDS * 1000UL) {
    mqtt_publishHeartbeat();
    lastHeartbeatMs = now;
  }
}

void mqtt_publishTelemetry() {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<512> doc;
  doc["id"] = settings.deviceID;
  doc["temp"] = state.temp;
  doc["hum"] = state.hum;
  doc["soil1"] = state.soil1;
  doc["soil2"] = state.soil2;
  doc["light"] = state.light;
  doc["ph"] = state.ph;
  doc["ip"] = WiFi.localIP().toString();
  doc["mac"] = WiFi.macAddress();
  doc["fw"] = FIRMWARE_VERSION;
  doc["freeHeap"] = (unsigned)ESP.getFreeHeap();
  doc["uptimeMs"] = millis();
  doc["pumpAuto"] = settings.pumpAuto;
  doc["fanAuto"] = settings.fanAuto;
  doc["lightAuto"] = settings.lightAuto;
  doc["relay_override"] = settings.relayOverride;
  char buf[512]; size_t n = serializeJson(doc, buf);
  mqttClient.publish(topicTelemetry.c_str(), (const uint8_t*)buf, n);
}

void mqtt_publishHeartbeat() {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<256> doc;
  doc["id"] = settings.deviceID;
  // use NTP epoch if available, otherwise millis-based seconds
  if (ntpSynced) doc["ts"] = (uint32_t)timeClient.getEpochTime();
  else doc["ts"] = (uint32_t)(millis() / 1000UL);
  doc["ip"] = WiFi.localIP().toString();
  doc["mac"] = WiFi.macAddress();
  doc["freeHeap"] = (unsigned)ESP.getFreeHeap();
  doc["uptimeMs"] = millis();
  char buf[256]; size_t n = serializeJson(doc, buf);
  mqttClient.publish(topicHeartbeat.c_str(), (const uint8_t*)buf, n);
}

// wifi_server.cpp
#include "config.h"
#include "sensors.h"
#include "relay_control.h"
#include "wifi_server.h"
#include "eeprom_utils.h"  // Add this for saveSettings
#include "ota_update.h"
#include "lcd_menu.h"
#include "mqtt_client.h"
#include <WebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <esp_sleep.h>

char scannedSSIDs[MAX_WIFI_NETWORKS][33];
int numScannedNetworks = 0;
int selectedWifiIndex = 0;
char selectedSSID[33] = {0};
WebServer* webServer = NULL;

// file-scoped reusable buffers to avoid large stack allocations in serverTask
static char g_responseBuf[1024];
static char g_payloadBuf[512];

// Telemetry queue - serverTask will process telemetry requests so network operations are isolated
typedef uint32_t TelemetryReq;
QueueHandle_t telemetryQueue = NULL;
// telemetry pending flag indicates there is a pending request to send
volatile bool telemetryPending = false;
// timestamp (ms) of last successful telemetry send
unsigned long lastTelemetrySent = 0;
// when true, include full settings in next telemetry and ask server to persist
volatile bool telemetryPersistConfig = false;

// Failed payload retry queue (in-memory ring buffer)
struct FailedPayload {
  char data[512];
  size_t len;
  uint32_t ts;
};
static FailedPayload failedQueue[5];
static int failedHead = 0;
static int failedTail = 0;
static int failedCount = 0;

// track LittleFS mount state to avoid repeated mount attempts
static bool littlefsAvailable = false;
static bool littlefsFormatTried = false;

static bool ensureLittleFS() {
  if (littlefsAvailable) return true;
  if (LittleFS.begin()) {
    littlefsAvailable = true;
    return true;
  }
  // do not auto-format LittleFS here — formatting would erase identity.json
  // Log the failure and return; formatting must be an explicit admin action.
  if (!littlefsFormatTried) {
    littlefsFormatTried = true;
    Serial.println("LittleFS mount failed; not formatting automatically.");
  }
  return false;
}

static void enqueueFailedPayload(const char* data, size_t len) {
  if (failedCount >= 5) return; // drop if full
  int idx = failedTail;
  memcpy(failedQueue[idx].data, data, len);
  failedQueue[idx].len = len;
  failedQueue[idx].ts = millis();
  failedTail = (failedTail + 1) % 5;
  failedCount++;
  // persist to LittleFS if available (avoid repeated errors when FS not mounted)
  if (ensureLittleFS()) {
    DynamicJsonDocument doc(2048);
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < failedCount; i++) {
      int p = (failedHead + i) % 5;
      arr.add(String(failedQueue[p].data).substring(0, failedQueue[p].len));
    }
    File f = LittleFS.open("/failed_payloads.json", "w");
    if (f) {
      serializeJson(doc, f);
      f.close();
    }
  }
}

static bool retryFailedPayloads() {
  if (failedCount == 0) return false;
  bool anySuccess = false;
  int attempts = failedCount;
  while (attempts-- > 0 && failedCount > 0) {
    int idx = failedHead;
    HTTPClient http2;
    char url2[128];
    snprintf(url2, sizeof(url2), "http://%s/api/v1/agents/%s/status", SERVER_IP, settings.deviceID);
    http2.begin(url2);
    http2.addHeader("Content-Type", "application/json");
    http2.addHeader("X-Device-Token", settings.token);
    int code2 = http2.POST((uint8_t*)failedQueue[idx].data, failedQueue[idx].len);
    http2.end();
    if (code2 == 200) {
      // remove head
      failedHead = (failedHead + 1) % 5;
      failedCount--;
      // persist updated queue
      DynamicJsonDocument doc(4096);
      JsonArray arr = doc.to<JsonArray>();
      for (int i = 0; i < failedCount; i++) {
        int p = (failedHead + i) % 5;
        arr.add(String(failedQueue[p].data).substring(0, failedQueue[p].len));
      }
      File f = LittleFS.open("/failed_payloads.json", "w");
      if (f) { serializeJson(doc, f); f.close(); }
      anySuccess = true;
    } else {
      // if still fails, stop retrying to avoid hammering
      break;
    }
  }
  return anySuccess;
}

// Load failed payloads from LittleFS into memory queue (call at boot)
void loadFailedQueueFromFS() {
  if (!ensureLittleFS()) return;
  if (!LittleFS.exists("/failed_payloads.json")) return;
  File f = LittleFS.open("/failed_payloads.json", "r");
  if (!f) return;
  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return;
  JsonArray arr = doc.as<JsonArray>();
  for (JsonVariant v : arr) {
    if (failedCount >= 5) break;
    const char* s = v.as<const char*>();
    size_t l = strlen(s);
    int idx = failedTail;
    memcpy(failedQueue[idx].data, s, l);
    failedQueue[idx].len = l;
    failedQueue[idx].ts = millis();
    failedTail = (failedTail + 1) % 5;
    failedCount++;
  }
}

void initTelemetryQueue() {
  if (!telemetryQueue) telemetryQueue = xQueueCreate(5, sizeof(TelemetryReq));
}

// Track whether an OTA download is currently active. Defined here so other modules
// can reference the state (`extern bool isOTARunning;` in headers).
bool isOTARunning = false;

void scanWiFiNetworks() {
  numScannedNetworks = WiFi.scanNetworks();
  if (numScannedNetworks == 0) {
    Serial.println("Khong tim thay mang WiFi nao");
    return;
  }

  numScannedNetworks = min(numScannedNetworks, MAX_WIFI_NETWORKS);
  for (int i = 0; i < numScannedNetworks; i++) {
    strncpy(scannedSSIDs[i], WiFi.SSID(i).c_str(), sizeof(scannedSSIDs[i]) - 1);
    scannedSSIDs[i][sizeof(scannedSSIDs[i]) - 1] = '\0';
    Serial.printf("%d: %s (RSSI: %d dBm)\n", i, scannedSSIDs[i], WiFi.RSSI(i));
  }
}

void initWatchdog() {
  Serial.println("Using internal watchdogTask for health checks (task WDT disabled)"); // disable task WDT
}

void feedWatchdog() {
  esp_task_wdt_reset();
}

void connectWiFi() {
  Serial.println("Reset WiFi state...");

  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(500);

  Serial.printf("SSID=[%s] len=%d\n", settings.ssid, strlen(settings.ssid));
  Serial.printf("PASS len=%d\n", strlen(settings.pass));

  WiFi.begin(settings.ssid, settings.pass);

  uint8_t attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    feedWatchdog();
    yield();
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Ket noi WiFi thanh cong");
    // Request LCD update via queue (do not access LCD directly)
    char l0[21];
    char l1[21];
    snprintf(l0, sizeof(l0), "WiFi OK           ");
    snprintf(l1, sizeof(l1), "%s", WiFi.localIP().toString().c_str());
    lcdPostLine(0, l0);
    lcdPostLine(1, l1);
    // start small debug web server if not already
    if (!webServer) {
      webServer = new WebServer(80);
      webServer->on("/debug", HTTP_GET, []() {
        char buf[512];
        size_t freeHeap = ESP.getFreeHeap();
        unsigned int sServer = serverTaskHandle ? uxTaskGetStackHighWaterMark(serverTaskHandle) : 0;
        unsigned int sLCD = lcdTaskHandle ? uxTaskGetStackHighWaterMark(lcdTaskHandle) : 0;
        unsigned int sBtn = buttonTaskHandle ? uxTaskGetStackHighWaterMark(buttonTaskHandle) : 0;
        unsigned int sSensor = sensorTaskHandle ? uxTaskGetStackHighWaterMark(sensorTaskHandle) : 0;
        snprintf(buf, sizeof(buf), "{\"freeHeap\":%u,\"serverStackHigh\":%u,\"lcdStackHigh\":%u,\"btnStackHigh\":%u,\"sensorStackHigh\":%u,\"uptimeMs\":%lu}", (unsigned)freeHeap, sServer, sLCD, sBtn, sSensor, millis());
        webServer->send(200, "application/json", buf);
      });
      // allow backend to push config immediately
      webServer->on("/apply_config", HTTP_POST, []() {
        String body = webServer->arg("plain");
        Serial.printf("[/apply_config] body len=%u\n", body.length());
        if (body.length() == 0) {
          webServer->send(400, "application/json", "{\"error\":\"empty body\"}");
          return;
        }
          // Use heap-allocated JSON to avoid stack overflow in serverTask
          DynamicJsonDocument doc(4096);
          DeserializationError err = deserializeJson(doc, body);
        if (err) {
          Serial.printf("[/apply_config] json error: %s\n", err.c_str());
          // return 400 so backend knows it's bad format
          webServer->send(400, "application/json", "{\"error\":\"invalid json\"}");
          return;
        }
        // For debugging: log a short preview of the body
        if (body.length() < 800) Serial.println(body);

        // If backend/admin posts {"ota": true} via /apply_config, trigger OTA locally for debugging
        if (doc.containsKey("ota") && doc["ota"] == true) {
          Serial.println("[/apply_config] ota=true received, calling performOTA() directly for debug");
          performOTA();
        }

        // Respect suppression window and on-device editing to avoid overwriting local edits
        bool suppress = (millis() < suppressRemoteUntil) || (menuState >= EDIT_TEMP && menuState <= EDIT_PH_MAX);
        if (suppress) Serial.println("[/apply_config] Suppressing remote apply due to local edit window");
        if (doc.containsKey("tempThresh") && !suppress) settings.tempThresh = doc["tempThresh"].as<float>();
        if (doc.containsKey("temperature_threshold") && !suppress) settings.tempThresh = doc["temperature_threshold"].as<float>();
        if (doc.containsKey("humThresh") && !suppress) settings.humThresh = doc["humThresh"].as<float>();
        if (doc.containsKey("humidity_threshold") && !suppress) settings.humThresh = doc["humidity_threshold"].as<float>();
        if (doc.containsKey("soilThresh") && !suppress) settings.soilThresh = doc["soilThresh"].as<float>();
        if (doc.containsKey("soil_threshold") && !suppress) settings.soilThresh = doc["soil_threshold"].as<float>();
        if (doc.containsKey("lightThresh") && !suppress) settings.lightThresh = doc["lightThresh"].as<float>();
        if (doc.containsKey("light_threshold") && !suppress) settings.lightThresh = doc["light_threshold"].as<float>();
        if (doc.containsKey("phThreshMin") && !suppress) settings.phThreshMin = doc["phThreshMin"].as<float>();
        if (doc.containsKey("phThreshMax") && !suppress) settings.phThreshMax = doc["phThreshMax"].as<float>();
        if (doc.containsKey("dailyWater")) settings.dailyWater = doc["dailyWater"].as<bool>();
        if (doc.containsKey("lightAuto") && !suppress) settings.lightAuto = doc["lightAuto"].as<bool>();
        if (doc.containsKey("light_auto") && !suppress) settings.lightAuto = doc["light_auto"].as<bool>();

        // Process auto-mode flags first so that a simultaneous request to disable autos
        // and enable relay_override in the same payload is honored.
        bool hasPumpAuto = doc.containsKey("pumpAuto") || doc.containsKey("pump_auto");
        bool hasFanAuto = doc.containsKey("fanAuto") || doc.containsKey("fan_auto");
        bool hasLightAuto = doc.containsKey("lightAuto") || doc.containsKey("light_auto");
        bool newPumpAuto = settings.pumpAuto;
        bool newFanAuto = settings.fanAuto;
        bool newLightAuto = settings.lightAuto;
        if (doc.containsKey("pumpAuto")) newPumpAuto = doc["pumpAuto"].as<bool>();
        if (doc.containsKey("pump_auto")) newPumpAuto = doc["pump_auto"].as<bool>();
        if (doc.containsKey("fanAuto")) newFanAuto = doc["fanAuto"].as<bool>();
        if (doc.containsKey("fan_auto")) newFanAuto = doc["fan_auto"].as<bool>();
        if (doc.containsKey("lightAuto")) newLightAuto = doc["lightAuto"].as<bool>();
        if (doc.containsKey("light_auto")) newLightAuto = doc["light_auto"].as<bool>();

        if (!suppress) {
          if (hasPumpAuto) { settings.pumpAuto = newPumpAuto; if (settings.pumpAuto) settings.relayOverride = false; }
          if (hasFanAuto) { settings.fanAuto = newFanAuto; if (settings.fanAuto) settings.relayOverride = false; }
          if (hasLightAuto) { settings.lightAuto = newLightAuto; if (settings.lightAuto) settings.relayOverride = false; }
        }
        if (doc.containsKey("addedToWeb")) settings.addedToWeb = doc["addedToWeb"].as<bool>();
        // relay override / manual relay control from backend
        if (doc.containsKey("relay_override")) {
          bool ro = doc["relay_override"].as<bool>();
          // Decide whether to honor remote override using the requested auto flags
          // If the incoming payload requests autos to be enabled, treat that as blocking override.
          bool requestedAutoActive = (hasPumpAuto ? newPumpAuto : settings.pumpAuto) || (hasFanAuto ? newFanAuto : settings.fanAuto) || (hasLightAuto ? newLightAuto : settings.lightAuto);
          if (ro && requestedAutoActive) {
            Serial.println("[/apply_config] Ignoring remote relay_override because Auto mode active (requested)");
          } else {
            settings.relayOverride = ro;
            if (settings.relayOverride) {
              settings.pumpAuto = false;
              settings.fanAuto = false;
              settings.lightAuto = false;
            }
          }
        }
        if (doc.containsKey("pump")) {
          bool p = doc["pump"].as<bool>();
          if (!settings.pumpAuto) { state.pump = p; digitalWrite(RELAY_PUMP, state.pump); }
          else Serial.println("[/apply_config] Ignoring remote pump command because pumpAuto is enabled");
        }
        if (doc.containsKey("fan")) {
          bool f = doc["fan"].as<bool>();
          if (!settings.fanAuto) { state.fan = f; digitalWrite(RELAY_FAN, state.fan); }
          else Serial.println("[/apply_config] Ignoring remote fan command because fanAuto is enabled");
        }
        if (doc.containsKey("lightOn")) {
          bool l = doc["lightOn"].as<bool>();
          if (!settings.lightAuto) { state.lightOn = l; digitalWrite(RELAY_LIGHT, state.lightOn); }
          else Serial.println("[/apply_config] Ignoring remote light command because lightAuto is enabled");
        }
        if (doc.containsKey("deepSleep")) {
          bool ds = doc["deepSleep"].as<bool>();
          settings.deepSleep = ds;
          if (ds) {
            saveSettingsNow();
            // send telemetry and enter deep sleep
            requestTelemetrySend();
            delay(300);
            esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_OK, 0);
            ESP.deepSleep(0);
          }
        }

        if (doc.containsKey("schedules") && !suppress) {
          JsonArray arr = doc["schedules"];
          settings.numSchedules = min((uint8_t)arr.size(), (uint8_t)MAX_SCHEDULES);
          for (uint8_t i = 0; i < settings.numSchedules; i++) {
            settings.schedules[i].hour = arr[i]["hour"];
            settings.schedules[i].minute = arr[i]["minute"];
            settings.schedules[i].forPump = arr[i]["forPump"];
            settings.schedules[i].forLight = arr[i]["forLight"];
          }
        }

        // MQTT runtime config: allow backend to set broker/port/credentials
        if (doc.containsKey("mqttBroker")) {
          const char* mb = doc["mqttBroker"];
          if (mb) {
            strncpy(settings.mqttBroker, mb, sizeof(settings.mqttBroker)-1);
            settings.mqttBroker[sizeof(settings.mqttBroker)-1] = '\0';
          }
        }
        if (doc.containsKey("mqttPort")) {
          settings.mqttPort = doc["mqttPort"].as<uint16_t>();
        }
        if (doc.containsKey("mqttUser")) {
          const char* mu = doc["mqttUser"];
          if (mu) {
            strncpy(settings.mqttUser, mu, sizeof(settings.mqttUser)-1);
            settings.mqttUser[sizeof(settings.mqttUser)-1] = '\0';
          }
        }
        if (doc.containsKey("mqttPass")) {
          const char* mp = doc["mqttPass"];
          if (mp) {
            strncpy(settings.mqttPass, mp, sizeof(settings.mqttPass)-1);
            settings.mqttPass[sizeof(settings.mqttPass)-1] = '\0';
          }
        }
        if (doc.containsKey("mqttUseTLS")) {
          settings.mqttUseTLS = doc["mqttUseTLS"].as<bool>();
        }

        // persist immediately to EEPROM so web changes survive restarts
        saveSettingsNow();
        // Re-init MQTT to pick up any updated runtime MQTT settings
        mqtt_init();
        controlRelays();
        // request immediate UI refresh (do not overwrite other menus)
        if (menuState == MAIN_SCREEN) drawMainScreen(); else needMainRefresh = true;
        webServer->send(200, "application/json", "{\"ok\":true}");
      });
      // allow remote reset via POST /reset (used by backend admin)
      webServer->on("/reset", HTTP_POST, []() {
        Serial.println("[/reset] reboot requested via HTTP");
        webServer->send(200, "application/json", "{\"ok\":true}");
        delay(100);
        ESP.restart();
      });
      webServer->begin();
      Serial.println("Debug web server started on port 80");
    }
  } else {
    Serial.println("Ket noi WiFi that bai");
    // Request LCD update via queue (do not access LCD directly)
    lcdPostLine(0, "WiFi Failed       ");
    lcdPostLine(1, settings.ssid);
  }
}

void handleServerComm() {
  if (WiFi.status() != WL_CONNECTED) return;

  feedWatchdog();
  // attempt to resend any failed payloads first (non-blocking, limited)
  retryFailedPayloads();

  HTTPClient http;
  http.setTimeout(10000);
  // Send telemetry to backend agent status endpoint and include device token for auth
  char url[160];
  // include explicit port so device connects to intended service
  snprintf(url, sizeof(url), "http://%s:%d/api/v1/agents/%s/status", SERVER_IP, SERVER_PORT, settings.deviceID);
  Serial.printf("[HTTP] POST %s\n", url);
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-Token", settings.token);
  // close connection after request to avoid keep-alive/socket reuse issues
  http.addHeader("Connection", "close");

  DynamicJsonDocument doc(768);
  doc["id"] = settings.deviceID;
  doc["temp"] = state.temp;
  doc["hum"] = state.hum;
  doc["soil1"] = state.soil1;
  doc["soil2"] = state.soil2;
  doc["light"] = state.light;
  doc["ph"] = state.ph;
  // include local IP and MAC so backend records correct reachable address and identity
  doc["ip"] = WiFi.localIP().toString();
  doc["mac"] = WiFi.macAddress();
  doc["fw"] = FIRMWARE_VERSION;
  doc["deepSleep"] = settings.deepSleep;
  doc["pump"] = state.pump;
  doc["fan"] = state.fan;
  doc["lightOn"] = state.lightOn;
  // include auto-mode flags and override state
  doc["pumpAuto"] = settings.pumpAuto;
  doc["fanAuto"] = settings.fanAuto;
  doc["lightAuto"] = settings.lightAuto;
  doc["relay_override"] = settings.relayOverride;
  // If requested, include full settings and ask server to persist them
  if (telemetryPersistConfig) {
    doc["persistConfig"] = true;
    doc["tempThresh"] = settings.tempThresh;
    doc["humThresh"] = settings.humThresh;
    doc["soilThresh"] = settings.soilThresh;
    doc["lightThresh"] = settings.lightThresh;
    doc["phThreshMin"] = settings.phThreshMin;
    doc["phThreshMax"] = settings.phThreshMax;
    doc["pumpAuto"] = settings.pumpAuto;
    doc["fanAuto"] = settings.fanAuto;
    doc["lightAuto"] = settings.lightAuto;
  }
  // health telemetry
  doc["freeHeap"] = (unsigned)ESP.getFreeHeap();
  doc["uptimeMs"] = millis();
  extern unsigned long eepromWriteCount; // declared in eeprom_utils.h
  doc["eepromWrites"] = eepromWriteCount;

  size_t len = serializeJson(doc, g_payloadBuf, sizeof(g_payloadBuf));
  // publish telemetry also via MQTT (best-effort)
  mqtt_publishTelemetry();
  Serial.printf("[HTTP] payload len=%u\n", (unsigned)len);
  int code = -1;
  // try twice on transient socket errors
  for (int attempt = 0; attempt < 2; attempt++) {
    code = http.POST((uint8_t*)g_payloadBuf, len);
    if (code > 0) break;
    Serial.printf("[HTTP] attempt %d failed, code=%d, err=%s\n", attempt+1, code, http.errorToString(code).c_str());
    delay(250);
  }
  feedWatchdog();

  if (code == 200) {
    // Read response into fixed buffer to avoid dynamic String
    WiFiClient* stream = http.getStreamPtr();
    size_t pos = 0;
    unsigned long start = millis();
    while ((millis() - start) < 1000 && pos < sizeof(g_responseBuf) - 1) {
      while (stream->available() && pos < sizeof(g_responseBuf) - 1) {
        g_responseBuf[pos++] = (char)stream->read();
      }
      if (!stream->connected()) break;
      delay(1);
    }
    g_responseBuf[pos] = '\0';
    feedWatchdog();

  // Use a heap-allocated JSON document to avoid large stack allocations
  DynamicJsonDocument respDoc(4096);
  DeserializationError error = deserializeJson(respDoc, g_responseBuf);
    feedWatchdog();
    yield();
    if (!error) {
      // For debugging: print full server response so we can see fields
      Serial.println("[SERVER_RESP] response: ");
      Serial.println(g_responseBuf);
      bool suppress = (millis() < suppressRemoteUntil) || (menuState >= EDIT_TEMP && menuState <= EDIT_PH_MAX);

        // If server sent a pending config block, apply it (same as top-level fields)
        if (respDoc.containsKey("pending")) {
          JsonObject p = respDoc["pending"].as<JsonObject>();
          Serial.println("[SERVER_RESP] pending config received");
          if (p.containsKey("tempThresh")) settings.tempThresh = p["tempThresh"].as<float>();
          if (p.containsKey("temperature_threshold")) settings.tempThresh = p["temperature_threshold"].as<float>();
          if (p.containsKey("humThresh")) settings.humThresh = p["humThresh"].as<float>();
          if (p.containsKey("humidity_threshold")) settings.humThresh = p["humidity_threshold"].as<float>();
          if (p.containsKey("soilThresh")) {
            float v = p["soilThresh"].as<float>();
            if (v != 0.0f) settings.soilThresh = v;
            else Serial.println("[SERVER_RESP] ignoring soilThresh==0.0 from pending");
          }
          if (p.containsKey("soil_threshold")) {
            float v = p["soil_threshold"].as<float>();
            if (v != 0.0f) settings.soilThresh = v;
            else Serial.println("[SERVER_RESP] ignoring soil_threshold==0.0 from pending");
          }
          if (p.containsKey("lightThresh")) settings.lightThresh = p["lightThresh"].as<float>();
          if (p.containsKey("light_threshold")) settings.lightThresh = p["light_threshold"].as<float>();
          if (p.containsKey("phThreshMin")) settings.phThreshMin = p["phThreshMin"].as<float>();
          if (p.containsKey("phThreshMax")) settings.phThreshMax = p["phThreshMax"].as<float>();
          if (p.containsKey("dailyWater")) settings.dailyWater = p["dailyWater"].as<bool>();
          if (p.containsKey("pumpAuto")) Serial.printf("[SERVER_RESP] pending.pumpAuto=%d\n", p["pumpAuto"].as<bool>());
          if (p.containsKey("pump_auto")) Serial.printf("[SERVER_RESP] pending.pump_auto=%d\n", p["pump_auto"].as<bool>());
          if (p.containsKey("fanAuto")) Serial.printf("[SERVER_RESP] pending.fanAuto=%d\n", p["fanAuto"].as<bool>());
          if (p.containsKey("fan_auto")) Serial.printf("[SERVER_RESP] pending.fan_auto=%d\n", p["fan_auto"].as<bool>());
          if (p.containsKey("lightAuto")) Serial.printf("[SERVER_RESP] pending.lightAuto=%d\n", p["lightAuto"].as<bool>());
          if (p.containsKey("light_auto")) Serial.printf("[SERVER_RESP] pending.light_auto=%d\n", p["light_auto"].as<bool>());
          if (!suppress && p.containsKey("pumpAuto")) { settings.pumpAuto = p["pumpAuto"].as<bool>(); if (settings.pumpAuto) settings.relayOverride = false; }
          if (!suppress && p.containsKey("pump_auto")) { settings.pumpAuto = p["pump_auto"].as<bool>(); if (settings.pumpAuto) settings.relayOverride = false; }
          if (!suppress && p.containsKey("fanAuto")) { settings.fanAuto = p["fanAuto"].as<bool>(); if (settings.fanAuto) settings.relayOverride = false; }
          if (!suppress && p.containsKey("fan_auto")) { settings.fanAuto = p["fan_auto"].as<bool>(); if (settings.fanAuto) settings.relayOverride = false; }
          if (!suppress && p.containsKey("lightAuto")) { settings.lightAuto = p["lightAuto"].as<bool>(); if (settings.lightAuto) settings.relayOverride = false; }
          if (!suppress && p.containsKey("light_auto")) { settings.lightAuto = p["light_auto"].as<bool>(); if (settings.lightAuto) settings.relayOverride = false; }
          if (p.containsKey("addedToWeb")) settings.addedToWeb = p["addedToWeb"].as<bool>();
          if (p.containsKey("schedules")) {
            JsonArray parr = p["schedules"];
            settings.numSchedules = min((uint8_t)parr.size(), (uint8_t)MAX_SCHEDULES);
            for (uint8_t i = 0; i < settings.numSchedules; i++) {
              settings.schedules[i].hour = parr[i]["hour"];
              settings.schedules[i].minute = parr[i]["minute"];
              settings.schedules[i].forPump = parr[i]["forPump"];
              settings.schedules[i].forLight = parr[i]["forLight"];
            }
          }
        }

      if (respDoc.containsKey("ota") && respDoc["ota"] == true) {
        Serial.println("[SERVER] OTA requested");
        Serial.println("[SERVER] about to call requestOTA()");
        requestOTA();
        Serial.println("[SERVER] returned from requestOTA()");
      }

      if (respDoc.containsKey("reset") && respDoc["reset"] == true) {
        Serial.println("[SERVER] Reset requested by server");
        // persist any settings and restart
        saveSettingsNow();
        delay(100);
        ESP.restart();
      }

      if (respDoc.containsKey("ssid") && respDoc.containsKey("pass")) {
        const char* newSSID = respDoc["ssid"];
        const char* newPASS = respDoc["pass"];

        if (newSSID != nullptr && strlen(newSSID) > 0 && strlen(newSSID) <= 31) {
          strncpy(settings.ssid, newSSID, sizeof(settings.ssid) - 1);
          settings.ssid[sizeof(settings.ssid)-1] = '\0';
          strncpy(settings.pass, newPASS ? newPASS : "", sizeof(settings.pass) - 1);
          settings.pass[sizeof(settings.pass)-1] = '\0';
          saveSettings(); 
          Serial.println("Updated WiFi from server");
          ESP.restart();
        }
      }

        if (!suppress && respDoc.containsKey("tempThresh")) settings.tempThresh = respDoc["tempThresh"].as<float>();
            // apply relay override if provided (but do not allow server to override when Auto mode is active)
            if (respDoc.containsKey("relay_override") && respDoc["relay_override"] == true) {
              if (settings.pumpAuto || settings.fanAuto || settings.lightAuto) {
                Serial.println("[SERVER] Ignoring relay_override because Auto mode active");
              } else {
                settings.relayOverride = true;
                if (respDoc.containsKey("pump")) { state.pump = respDoc["pump"].as<bool>(); digitalWrite(RELAY_PUMP, state.pump); }
                if (respDoc.containsKey("fan"))  { state.fan = respDoc["fan"].as<bool>(); digitalWrite(RELAY_FAN, state.fan); }
                if (respDoc.containsKey("lightOn")) { state.lightOn = respDoc["lightOn"].as<bool>(); digitalWrite(RELAY_LIGHT, state.lightOn); }
              }
            }

            if (respDoc.containsKey("deepSleep")) {
              bool ds = respDoc["deepSleep"].as<bool>();
              settings.deepSleep = ds;
              if (ds) {
                saveSettingsNow();
                // schedule deep sleep shortly
                Serial.println("[SERVER] DeepSleep requested by server, entering sleep...");
                delay(200);
                // send telemetry and enter deep sleep
                requestTelemetrySend();
                delay(500);
                esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_OK, 0); // wake on OK pressed (active LOW)
                ESP.deepSleep(0);
              }
            }
      if (!suppress) {
        if (respDoc.containsKey("humThresh")) settings.humThresh = respDoc["humThresh"].as<float>();
        if (respDoc.containsKey("soilThresh")) {
          float v = respDoc["soilThresh"].as<float>();
          if (v != 0.0f) settings.soilThresh = v; else Serial.println("[SERVER_RESP] ignoring soilThresh==0.0");
        }
        if (respDoc.containsKey("lightThresh")) settings.lightThresh = respDoc["lightThresh"].as<float>();
        if (respDoc.containsKey("phThreshMin")) settings.phThreshMin = respDoc["phThreshMin"].as<float>();
        if (respDoc.containsKey("phThreshMax")) settings.phThreshMax = respDoc["phThreshMax"].as<float>();
      }
      if (respDoc.containsKey("dailyWater")) settings.dailyWater = respDoc["dailyWater"].as<bool>();
      if (respDoc.containsKey("lightAuto") && !suppress) settings.lightAuto = respDoc["lightAuto"].as<bool>();
      if (respDoc.containsKey("light_auto") && !suppress) settings.lightAuto = respDoc["light_auto"].as<bool>();
      if (respDoc.containsKey("addedToWeb")) settings.addedToWeb = respDoc["addedToWeb"].as<bool>();

      if (respDoc.containsKey("schedules")) {
        JsonArray arr = respDoc["schedules"];
        settings.numSchedules = min((uint8_t)arr.size(), (uint8_t)MAX_SCHEDULES);
        for (uint8_t i = 0; i < settings.numSchedules; i++) {
          settings.schedules[i].hour = arr[i]["hour"];
          settings.schedules[i].minute = arr[i]["minute"];
          settings.schedules[i].forPump = arr[i]["forPump"];
          settings.schedules[i].forLight = arr[i]["forLight"];
        }
      }
      // Also accept snake_case top-level keys from server
      if (!suppress && respDoc.containsKey("temperature_threshold")) settings.tempThresh = respDoc["temperature_threshold"].as<float>();
      if (!suppress && respDoc.containsKey("humidity_threshold")) settings.humThresh = respDoc["humidity_threshold"].as<float>();
      if (!suppress && respDoc.containsKey("light_threshold")) settings.lightThresh = respDoc["light_threshold"].as<float>();
      if (!suppress && respDoc.containsKey("soil_threshold")) {
        float v = respDoc["soil_threshold"].as<float>();
        if (v != 0.0f) settings.soilThresh = v; else Serial.println("[SERVER_RESP] ignoring soil_threshold==0.0");
      }
      if (!suppress && respDoc.containsKey("pump_auto")) { settings.pumpAuto = respDoc["pump_auto"].as<bool>(); if (settings.pumpAuto) settings.relayOverride = false; }
      if (!suppress && respDoc.containsKey("fan_auto")) { settings.fanAuto = respDoc["fan_auto"].as<bool>(); if (settings.fanAuto) settings.relayOverride = false; }
      if (!suppress && respDoc.containsKey("light_auto")) { settings.lightAuto = respDoc["light_auto"].as<bool>(); if (settings.lightAuto) settings.relayOverride = false; }
      saveSettings();
      controlRelays();
      if (menuState == MAIN_SCREEN) drawMainScreen(); else needMainRefresh = true;
      // clear persist flag after server responded
      telemetryPersistConfig = false;
    }
  } else {
    Serial.printf("HTTP POST failed, code: %d, err=%s\n", code, http.errorToString(code).c_str());
    // enqueue payload for retry later
    enqueueFailedPayload(g_payloadBuf, len);
  }
  http.end();
}

void requestTelemetrySend() {
  // mark pending so serverTask will attempt to send when allowed
  telemetryPending = true;
  if (!telemetryQueue) return;
  TelemetryReq r = (TelemetryReq)millis();
  xQueueSend(telemetryQueue, &r, 0);
}

// Request telemetry and request server to persist current settings
void requestTelemetrySendPersist() {
  telemetryPersistConfig = true;
  telemetryPending = true;
  if (!telemetryQueue) return;
  TelemetryReq r = (TelemetryReq)millis();
  xQueueSend(telemetryQueue, &r, 0);
}

void watchdogTask(void* pv) {
  const TickType_t delayTicks = pdMS_TO_TICKS(30000); // 30s check
  while (1) {
    unsigned int sServer = serverTaskHandle ? uxTaskGetStackHighWaterMark(serverTaskHandle) : 0;
    unsigned int sLCD = lcdTaskHandle ? uxTaskGetStackHighWaterMark(lcdTaskHandle) : 0;
    unsigned int sBtn = buttonTaskHandle ? uxTaskGetStackHighWaterMark(buttonTaskHandle) : 0;
    unsigned int sSensor = sensorTaskHandle ? uxTaskGetStackHighWaterMark(sensorTaskHandle) : 0;
    Serial.printf("[WATCHDOG] stacks: server=%u lcd=%u btn=%u sensor=%u freeHeap=%u\n", sServer, sLCD, sBtn, sSensor, (unsigned)ESP.getFreeHeap());
    // If any stack high-water is too small, force restart to recover
    const unsigned int STACK_THRESHOLD = 100;
    if ((sServer > 0 && sServer < STACK_THRESHOLD) || (sLCD > 0 && sLCD < STACK_THRESHOLD) ||
        (sBtn > 0 && sBtn < STACK_THRESHOLD) || (sSensor > 0 && sSensor < STACK_THRESHOLD)) {
      Serial.println("[WATCHDOG] Low stack detected, restarting ESP...");
      delay(200);
      ESP.restart();
    }
    vTaskDelay(delayTicks);
  }
}
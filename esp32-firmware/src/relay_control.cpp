// relay_control.cpp (đầy đủ)
#include "config.h"
#include "sensors.h"
#include "relay_control.h"
#include "wifi_server.h"

unsigned long pumpStartTime = 0;
unsigned long fanStartTime = 0;
unsigned long lightStartTime = 0;
// Use 10 seconds as requested for pump/fan on-duration
const unsigned long RELAY_DURATION = 10000;   // 10 giây
const unsigned long FAN_COOLDOWN = 30000;    // 30 giây cooldown cho quạt
unsigned long fanCooldownEnd = 0;
const unsigned long RELAY_MIN_TOGGLE_MS = 5000; // minimal interval between toggles for any relay
static unsigned long lastPumpToggle = 0; 
static unsigned long lastFanToggle = 0;
static unsigned long lastLightToggle = 0;

void initRelays() {
  pinMode(RELAY_PUMP, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT);
  pinMode(RELAY_LIGHT, OUTPUT);
  digitalWrite(RELAY_PUMP, LOW);
  digitalWrite(RELAY_FAN, LOW);
  digitalWrite(RELAY_LIGHT, LOW);
}

void controlRelays() {
  unsigned long now = millis();
  static unsigned long lastAlert = 0;
  if (millis() - lastAlert > 60000) {lastAlert = millis();}

  // Snapshot previous relay states to detect changes
  bool prevPump = state.pump;
  bool prevFan = state.fan;
  bool prevLight = state.lightOn;

  // Bơm (auto hoặc manual).
  // Use average of two soil sensors for decision when in auto mode.
  float avgSoil = ((state.soil1 + state.soil2) / 2.0);
  bool needPump = (!settings.relayOverride) && settings.pumpAuto && (avgSoil < settings.soilThresh);
  // Turn on pump when auto needs it (respect minimal toggle interval)
  if (needPump && !state.pump) {
    if (now - lastPumpToggle >= RELAY_MIN_TOGGLE_MS) {
      state.pump = true;
      digitalWrite(RELAY_PUMP, HIGH);
      lastPumpToggle = now;
      pumpStartTime = now;
    }
  }
  // Auto-off pump after RELAY_DURATION when in auto mode
  if (settings.pumpAuto && state.pump && pumpStartTime != 0 && now - pumpStartTime >= RELAY_DURATION) {
    if (now - lastPumpToggle >= RELAY_MIN_TOGGLE_MS) {
      state.pump = false;
      digitalWrite(RELAY_PUMP, LOW);
      lastPumpToggle = now;
    }
    pumpStartTime = 0;
  }

  // If pump state changed, notify server immediately and log
  if (state.pump != prevPump) {
    Serial.printf("[RELAY] pump -> %s\n", state.pump ? "ON" : "OFF");
    requestTelemetrySend();
  }

  bool needFan = (!settings.relayOverride) && settings.fanAuto && (state.temp > settings.tempThresh);
  // Turn on fan when needed (respect cooldown and minimal toggle interval)
  if (needFan && !state.fan && now > fanCooldownEnd) {
    if (now - lastFanToggle >= RELAY_MIN_TOGGLE_MS) {
      state.fan = true;
      digitalWrite(RELAY_FAN, HIGH);
      lastFanToggle = now;
      fanStartTime = now;
    }
  }
  // Auto-off fan after RELAY_DURATION when in auto mode
  if (settings.fanAuto && state.fan && fanStartTime != 0 && now - fanStartTime >= RELAY_DURATION) {
    if (now - lastFanToggle >= RELAY_MIN_TOGGLE_MS) {
      state.fan = false;
      digitalWrite(RELAY_FAN, LOW);
      lastFanToggle = now;
    }
    fanStartTime = 0;
    fanCooldownEnd = now + FAN_COOLDOWN;
  }

  if (state.fan != prevFan) {
    Serial.printf("[RELAY] fan -> %s\n", state.fan ? "ON" : "OFF");
    requestTelemetrySend();
  }

  // Light auto control
  bool needLight = (!settings.relayOverride) && settings.lightAuto && (state.light < settings.lightThresh);
  if (needLight && !state.lightOn) {
    // allow immediate turn-ON for light (no minimal toggle cooldown when turning on)
    state.lightOn = true;
    digitalWrite(RELAY_LIGHT, HIGH);
    lastLightToggle = now;
    lightStartTime = now;
  }
  // Auto-off light: prefer turning off only when a NEW sensor measurement exceeds threshold
  if (settings.lightAuto && state.lightOn) {
    // If we have a new light measurement taken after the light was started
    if (lastLightMeasured > lightStartTime && state.light > settings.lightThresh) {
      if (now - lastLightToggle >= RELAY_MIN_TOGGLE_MS) {
        state.lightOn = false;
        digitalWrite(RELAY_LIGHT, LOW);
        lastLightToggle = now;
      }
      lightStartTime = 0;
    } else if (lightStartTime != 0 && now - lightStartTime >= RELAY_DURATION) {
      // fallback: if duration elapsed without a new crossing, still allow auto-off
      if (now - lastLightToggle >= RELAY_MIN_TOGGLE_MS) {
        state.lightOn = false;
        digitalWrite(RELAY_LIGHT, LOW);
        lastLightToggle = now;
      }
      lightStartTime = 0;
    }
  }

  if (state.lightOn != prevLight) {
    Serial.printf("[RELAY] light -> %s\n", state.lightOn ? "ON" : "OFF");
    requestTelemetrySend();
  }

  // Warning alert for pH out of range every 60 seconds
  if (WiFi.status() == WL_CONNECTED &&
      (state.ph < settings.phThreshMin || state.ph > settings.phThreshMax)) {

    HTTPClient http;
    char url[128];
    snprintf(url, sizeof(url), "http://%s:%d/alert", SERVER_IP, SERVER_PORT);
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    // close connection explicitly to avoid keep-alive reuse issues
    http.addHeader("Connection", "close");
    // include device token if available (backend will ignore if not required)
    http.addHeader("X-Device-Token", settings.token);

    StaticJsonDocument<256> doc;
    doc["id"] = settings.deviceID;
    char alertMsg[64];
    snprintf(alertMsg, sizeof(alertMsg), "pH out of range: %.1f", state.ph);
    doc["alert"] = alertMsg;

    char payload[256];
    size_t plen = serializeJson(doc, payload, sizeof(payload));
    http.setTimeout(6000); // slighty larger timeout
    int code = http.POST((uint8_t*)payload, plen);
    feedWatchdog();
    yield();
    if (code <= 0) {
      Serial.printf("[ALERT] POST failed code=%d err=%s\n", code, http.errorToString(code).c_str());
    }
    http.end();
  }
  
  checkSchedules();
}

void checkSchedules() {
  if (!ntpSynced) return;
  uint8_t h = timeClient.getHours();
  uint8_t m = timeClient.getMinutes();

  for (uint8_t i = 0; i < settings.numSchedules; i++) {
    Schedule s = settings.schedules[i];
    if (h == s.hour && m == s.minute) {
      if (s.forPump && !state.pump) {
        state.pump = true;
        digitalWrite(RELAY_PUMP, HIGH);
        pumpStartTime = millis();
      }
      if (s.forLight && !state.lightOn) {
        state.lightOn = true;
        digitalWrite(RELAY_LIGHT, HIGH);
        lightStartTime = millis();
      }
    }
  }
}
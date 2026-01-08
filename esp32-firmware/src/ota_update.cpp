#include "ota_update.h"
#include "config.h"
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFi.h>
#include <Update.h>

// Stream-download firmware and report progress back to server via HTTP
// Improved streaming OTA logic
// - sets isOTARunning while active
// - verifies Update.write return values
// - periodically checks server agent status to allow cancel
// - reports progress via post_ota_progress

extern bool isOTARunning; // declared in wifi_server.cpp
extern void feedWatchdog();

static void post_ota_progress(const char* device_id, int progress, const char* status) {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  char url[256];
  // Use POST with query params for compatibility with backend report handler
  snprintf(url, sizeof(url), "http://%s:%d/api/v1/firmware/report?device_id=%s&progress=%d&status=%s", SERVER_IP, SERVER_PORT, device_id, progress, status);
  http.begin(url);
  http.addHeader("Connection", "close");
  http.addHeader("X-Device-Token", settings.token);
  http.POST("");
  http.end();
}

volatile bool otaRequested = false;

void requestOTA() {
  otaRequested = true;
  Serial.println("[OTA] requestOTA called");
}

// Simple check to see whether server has cleared the OTA flag for this device.
// This is best-effort and only intended to support canceling an ongoing download
// when an operator presses Cancel in the UI.
static bool checkServerCancel(const char* device_id) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  char url[200];
  snprintf(url, sizeof(url), "http://%s:%d/api/v1/agents/%s/status", SERVER_IP, SERVER_PORT, device_id);
  http.begin(url);
  http.addHeader("X-Device-Token", settings.token);
  int code = http.GET();
  if (code == 200) {
    String body = http.getString();
    http.end();
    // If server responds with ota:false or no ota true, treat as cancel
    if (body.indexOf("\"ota\":true") < 0) return true;
    return false;
  }
  http.end();
  return false;
}

void performOTA() {
  Serial.printf("[OTA] performOTA entry otaRequested=%d\n", otaRequested ? 1 : 0);
  if (!otaRequested) return;
  otaRequested = false;

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA] WiFi not connected");
    return;
  }

  isOTARunning = true;
  post_ota_progress(settings.deviceID, 0, "started");

  Serial.println("[OTA] Starting streaming OTA update...");
  HTTPClient http;
  char url[256];
  snprintf(url, sizeof(url), "http://%s:%d/api/v1/firmware/latest", SERVER_IP, SERVER_PORT);
  http.begin(url);
  http.setTimeout(15000);
  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("[OTA] HTTP GET failed with code %d\n", httpCode);
    http.end();
    post_ota_progress(settings.deviceID, 0, "failed");
    isOTARunning = false;
    return;
  }

  int contentLength = http.getSize();
  WiFiClient * stream = http.getStreamPtr();

  if (contentLength <= 0) {
    Serial.println("[OTA] Unknown content length, aborting OTA");
    http.end();
    post_ota_progress(settings.deviceID, 0, "failed");
    isOTARunning = false;
    return;
  }

  if (!Update.begin((size_t)contentLength)) {
    Serial.println("[OTA] Update.begin failed (not enough space?)");
    http.end();
    post_ota_progress(settings.deviceID, 0, "failed");
    isOTARunning = false;
    return;
  }

  const size_t buffSize = 1024;
  uint8_t buff[buffSize];
  int written = 0;
  unsigned long lastReport = millis();
  unsigned long lastCancelCheck = millis();

  while (written < contentLength) {
    // allow watchdog and other tasks to run
    if (millis() - lastReport > 500) {
      feedWatchdog();
      lastReport = millis();
    }

    // Periodically poll server for cancel requests (best-effort)
    if (millis() - lastCancelCheck > 2000) {
      if (checkServerCancel(settings.deviceID)) {
        Serial.println("[OTA] Server cleared OTA flag — cancelling download");
        Update.abort();
        post_ota_progress(settings.deviceID, (int)((written * 100) / max(1, contentLength)), "cancelled");
        http.end();
        isOTARunning = false;
        return;
      }
      lastCancelCheck = millis();
    }

    size_t available = stream->available();
    if (available) {
      size_t toRead = available;
      if (toRead > buffSize) toRead = buffSize;
      int r = stream->readBytes(buff, toRead);
      if (r <= 0) break;
      size_t wrote = Update.write(buff, r);
      if (wrote != (size_t)r) {
        Serial.printf("[OTA] Write mismatch wrote=%u expected=%d\n", (unsigned)wrote, r);
        Update.abort();
        post_ota_progress(settings.deviceID, (int)((written * 100) / max(1, contentLength)), "failed");
        http.end();
        isOTARunning = false;
        return;
      }
      written += r;

      // report every 1s or on completion
      unsigned long now = millis();
      if (now - lastReport > 1000 || written >= contentLength) {
        int pct = (int)((written * 100) / contentLength);
        Serial.printf("[OTA] progress %d%% (%d/%d)\n", pct, written, contentLength);
        post_ota_progress(settings.deviceID, pct, "downloading");
        lastReport = now;
      }
    } else {
      delay(10);
    }
  }

  bool ok = false;
  if (written == contentLength) {
    ok = Update.end(true);
    if (ok) {
      Serial.println("[OTA] Update OK, rebooting...");
      post_ota_progress(settings.deviceID, 100, "done");
      http.end();
      delay(200);
      isOTARunning = false; // allow graceful state transition before restart
      ESP.restart();
      return;
    }
  }

  Serial.println("[OTA] Update failed during write");
  Update.abort();
  post_ota_progress(settings.deviceID, (int)((written * 100) / max(1, contentLength)), "failed");
  http.end();
  isOTARunning = false;
}

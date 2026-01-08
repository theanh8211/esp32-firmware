// main.ino (copy of sensors.ino to satisfy arduino-cli sketch name requirement)
#include "lcd_menu.h"
#include "config.h"
#include "sensors.h"
#include "relay_control.h"
#include "wifi_server.h"
#include "eeprom_utils.h"
#include "ota_update.h"
// #define CLEAR_EEPROM_ONCE   // clear EEPROM
hd44780_I2Cexp lcd;

static bool otaInitialized = false;
static bool ntpInitialized = false;
bool lcdAvailable = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== BOOT START ===");
  Serial.printf("FIRMWARE_VERSION=%s\n", FIRMWARE_VERSION);
  initEEPROM();

//clear EEPROM
#ifdef CLEAR_EEPROM_ONCE
  Serial.println("CLEAR EEPROM ONCE");
  clearEEPROM();
  while (1);
#endif

  initWatchdog();


  Wire.begin(SDA_PIN, SCL_PIN);
  // Init filesystem for persistent payload queue
  if (!LittleFS.begin()) {
    Serial.println("LittleFS failed to mount");
  } else {
    Serial.println("LittleFS mounted");
  }

  int status = lcd.begin(20, 4);
  if (status) {
    Serial.print("LCD init failed: ");
    Serial.println(status);
    lcdAvailable = false;
  } else {
    // create LCD mutex before any task uses it
    lcdMutex = xSemaphoreCreateMutex();

    lcd.backlight();
    lcd.clear();
    lcd.print("Smart Farm VIET NAM");
    Serial.println("LCD khoi tao thanh cong voi hd44780");
    lcdAvailable = true;
  }

  loadSettings();
  initButtons();
  initRelays();
  initSensors();
  connectWiFi();
  // initialize MQTT client (will attempt connect if WiFi available)
  mqtt_init();
  // start network/server task pinned to core 0
  xTaskCreatePinnedToCore(
    serverTask,
    "ServerTask",
    12288,
    NULL,
    1,
    &serverTaskHandle,
    0   // core 0
  );

  // init telemetry queue and start watchdog task on core 0
  initTelemetryQueue();
  xTaskCreatePinnedToCore(watchdogTask, "WatchdogTask", 4096, NULL, 4, NULL, 0);
  // load any persisted failed telemetry payloads
  loadFailedQueueFromFS();

  // start UI and sensor tasks (they run on core 1)
  startUITasks();
  startSensorTask();
}

void serverTask(void *param) {
  uint32_t req;
  while (1) {
    feedWatchdog();
    // wait for telemetry requests (1s timeout) and process when arrived
    if (telemetryQueue && xQueueReceive(telemetryQueue, &req, pdMS_TO_TICKS(1000)) == pdPASS) {
      // mark pending; serverTask will honor throttle and send when allowed
      telemetryPending = true;
    }
    // handle debug web server client
    if (webServer) webServer->handleClient();
    // MQTT background loop
    mqtt_loop();
    // attempt to send telemetry if pending and throttle allows
    if (telemetryPending && WiFi.status() == WL_CONNECTED && (millis() - lastTelemetrySent >= 2000)) {
      handleServerComm();
      lastTelemetrySent = millis();
      telemetryPending = false;
    }
    // If OTA was requested by server response, perform it here so download runs in serverTask context
    // Only call performOTA when a request flag is set to avoid noisy polling logs
    if (otaRequested) performOTA();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void loop() {
  // Main loop now minimal - perform light housekeeping such as NTP and WiFi retry
  feedWatchdog();
  unsigned long now = millis();

  // Retry WiFi mỗi 30 giây nếu mất kết nối
  static unsigned long lastWiFiCheck = 0;
  if (now - lastWiFiCheck > 30000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi mat ket noi, thu lai...");
      WiFi.disconnect(true);
      delay(1000);
      connectWiFi();
    }
    lastWiFiCheck = now;
  }

  // NTP handling remains here
  bool connected = (WiFi.status() == WL_CONNECTED);
  if (connected) {
    if (!ntpInitialized) {
      timeClient.begin();
      ntpInitialized = true;
    }
    static unsigned long lastNTP = 0;
    if (!ntpSynced && now - lastNTP > 10000) {
      ntpSynced = timeClient.update();
      lastNTP = now;
    }
  }

  vTaskDelay(pdMS_TO_TICKS(200));
}


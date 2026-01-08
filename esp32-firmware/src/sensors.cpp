// sensors.cpp
#include "config.h"
#include "relay_control.h"
#include "wifi_server.h"

uint8_t dhtFailCount = 0;

bool dhtError;

DHT dht(DHT_PIN, DHT_TYPE);

struct SensorState {
  float temp = 0.0;
  float hum = 0.0;
  int soil1 = 0;
  int soil2 = 0;
  int light = 0;
  float ph = 7.0;
  bool pump = false;
  bool fan = false;
  bool lightOn = false;
};
SensorState state;
unsigned long lastLightMeasured = 0;

// forward declaration
void readSensors();

void initSensors() {
  dht.begin();
  pinMode(SOIL1_PIN, INPUT);
  pinMode(SOIL2_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(PH_PIN, INPUT);
  // Configure ADC for more stable readings
  analogSetPinAttenuation(SOIL1_PIN, ADC_11db);
  analogSetPinAttenuation(SOIL2_PIN, ADC_11db);
  analogSetPinAttenuation(LDR_PIN, ADC_11db);
  analogSetPinAttenuation(PH_PIN, ADC_11db);
  analogSetWidth(12); // 12-bit ADC (0-4095)
}

// Sensor task that periodically reads sensors every 3s
void sensorTask(void *param) {
  const TickType_t delayTicks = pdMS_TO_TICKS(3000);
  unsigned long lastTelemetry = 0;
  while (1) {
    feedWatchdog();
    readSensors();
    // after reading sensors, evaluate relay control logic
    controlRelays();
    // send telemetry at most once every 10s
    if (millis() - lastTelemetry >= 10000) {
      requestTelemetrySend();
      lastTelemetry = millis();
    }
    vTaskDelay(delayTicks);
  }
}

void startSensorTask() {
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 3, &sensorTaskHandle, 1);
}

void readSensors() {
  static unsigned long last = 0;
  if (millis() - last < 5000) return;
  last = millis();

  // record the time of this measurement (used by relay control to detect "new" readings)
  lastLightMeasured = last;

  // state.temp = dht.readTemperature();
  // state.hum = dht.readHumidity();
  // if (isnan(state.temp)) state.temp = 0; //nếu lỗi NaN thì giá trị = 0 --> nguy hiểm 
  // if (isnan(state.hum)) state.hum = 0;

  float t = dht.readTemperature();
  float h = dht.readHumidity();

  //nếu DHT11 đọc lỗi 5 lần -> print ERR 
  if (isnan(t) || isnan(h)) {
    dhtFailCount++;
  } else {
    dhtFailCount = 0;
  }

  if (!isnan(t)) {state.temp = t;}
  if (!isnan(h)) {state.hum = h;}

  // Read analog pins with averaging to reduce noise
  const int SAMPLES = 8;
  long sumSoil1 = 0, sumSoil2 = 0, sumLight = 0, sumPH = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sumSoil1 += analogRead(SOIL1_PIN);
    sumSoil2 += analogRead(SOIL2_PIN);
    sumLight += analogRead(LDR_PIN);
    sumPH += analogRead(PH_PIN);
    delay(2);
  }
  int rawSoil1 = sumSoil1 / SAMPLES;
  int rawSoil2 = sumSoil2 / SAMPLES;
  int rawLight = sumLight / SAMPLES;
  float rawPH = (float)sumPH / SAMPLES;

  Serial.printf("[SENSORS] DHT t=%.2f h=%.2f dhtFail=%u\n", t, h, dhtFailCount);
  Serial.printf("[SENSORS] RAW soil1=%d soil2=%d\n", rawSoil1, rawSoil2);
  Serial.printf("[SENSORS] rawLight=%d rawPHraw=%0.1f\n", rawLight, rawPH);

  // Map soil sensors to 0-100% with simple calibration/clamping (tweak MIN/MAX for your probes)
  const int SOIL_MIN = 1000;   // adjust to your dry reading
  const int SOIL_MAX = 3800;  // adjust to your wet reading
  int mappedSoil1 = constrain(map(rawSoil1, SOIL_MAX, SOIL_MIN, 0, 100), 0, 100);
  int mappedSoil2 = constrain(map(rawSoil2, SOIL_MAX, SOIL_MIN, 0, 100), 0, 100);

  // Map light to 0-100% (calibrate if needed)
  // Note: the sensor returns higher ADC when it's darker and lower ADC when brighter.
  // We invert the mapping so a brighter environment yields a larger percentage.
  const int LIGHT_MIN = 0;
  const int LIGHT_MAX = 4095;
  int percent = 0;
  if (LIGHT_MAX != LIGHT_MIN) {
    percent = (int)(((long)rawLight - LIGHT_MIN) * 100L / (LIGHT_MAX - LIGHT_MIN));
  }
  int mappedLight = constrain(100 - percent, 0, 100);

  // Convert averaged PH ADC to voltage then to pH with calibration constants
  float voltage = (rawPH * 3.3f) / 4095.0f;
  float computedPH = 7.0f + ((2.5f - voltage) / 0.18f);

  // Apply simple exponential smoothing to reduce jitter
  const float ALPHA = 0.3f; // smoothing factor (0..1)
  state.soil1 = (int)(ALPHA * mappedSoil1 + (1.0f - ALPHA) * state.soil1);
  state.soil2 = (int)(ALPHA * mappedSoil2 + (1.0f - ALPHA) * state.soil2);
  state.light = (int)(ALPHA * mappedLight + (1.0f - ALPHA) * state.light);
  // mark when light value was last updated (useful to ensure we act on a new reading)
  lastLightMeasured = last;
  state.ph = ALPHA * computedPH + (1.0f - ALPHA) * state.ph;

  Serial.printf("[SENSORS] light=%d ph=%.2f voltage=%.3f\n", state.light, state.ph, voltage);
}

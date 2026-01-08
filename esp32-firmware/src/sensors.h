// sensors.h
#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

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
extern SensorState state;
extern unsigned long lastLightMeasured;

void initSensors();
void readSensors();

// Start sensor FreeRTOS task (periodic read)
void startSensorTask();

#endif
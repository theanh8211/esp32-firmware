#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <Arduino.h>

void mqtt_init();
void mqtt_loop();
void mqtt_publishTelemetry();
void mqtt_publishHeartbeat();

#endif

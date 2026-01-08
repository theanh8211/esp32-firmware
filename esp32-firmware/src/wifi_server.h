// wifi_server.h
#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H
#pragma once
#include "config.h"
#include <WebServer.h>

void serverTask(void* pv);

void watchdogTask(void* pv);

// enqueue a telemetry send request (non-blocking)
void requestTelemetrySend();
void requestTelemetrySendPersist();

void initTelemetryQueue();
extern QueueHandle_t telemetryQueue;
// telemetry pending flag set by requestTelemetrySend
extern volatile bool telemetryPending;
// timestamp of last telemetry send (ms)
extern unsigned long lastTelemetrySent;
// when true, include full settings in next telemetry and ask server to persist
extern volatile bool telemetryPersistConfig;

// restore failed payload queue from LittleFS (call at boot)
void loadFailedQueueFromFS();

void connectWiFi();
void initWatchdog();
void feedWatchdog();
void handleServerComm();

// expose web server pointer for serverTask to call handleClient
extern WebServer* webServer;

#endif

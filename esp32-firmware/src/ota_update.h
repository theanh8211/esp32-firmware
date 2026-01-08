#pragma once
#include <Arduino.h>

extern volatile bool otaRequested;

void requestOTA();
void performOTA();

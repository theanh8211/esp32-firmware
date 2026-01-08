#ifndef EEPROM_UTILS_H
#define EEPROM_UTILS_H

#include "config.h"

void initEEPROM();
void loadSettings();
void saveSettings();
void saveSettingsNow();
void clearEEPROM(); 
extern unsigned long eepromWriteCount;

#endif
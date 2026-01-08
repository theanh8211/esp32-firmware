#ifndef LCD_MENU_H
#define LCD_MENU_H

#include "config.h"

void initLCD();
void initButtons();

// Start UI-related FreeRTOS tasks (button polling + LCD render)
void startUITasks();

// Post a single 20-char line to be shown by the lcd task (non-blocking)
void lcdPostLine(int row, const char* text);

void handleButtons();
void updateMenu();

void drawMainScreen();
void drawMainMenu();
void drawScheduleMenu();
void drawEditSchedule();
void drawThresholdMenu();
// draw edit view: show current value and editable value
void drawEditThreshold(float currentVal, float editVal, const char* label);
void drawLightSetMenu();
void drawVersionMenu();
void drawInfoMenu();
void drawManualControl();
void drawWiFiSetup();

void handleOK();
void handleBack();
void handleUp();
void handleDown();
void handleLeft();
void handleRight();

#endif

#include "lcd_menu.h"
#include "config.h"
#include "sensors.h"
#include "ota_update.h"
#include "eeprom_utils.h"
#include "wifi_server.h"
#include <esp_sleep.h>

extern hd44780_I2Cexp lcd;

// initialize lastState to an invalid value so first update forces a full redraw
static MenuState lastState = (MenuState)(-1);
static int lastMenuIndex = -1;
static int lastSubIndex = -1;

int menuIndex = 0;
int subIndex = 0;
int editIndex = 0;
int inputPos = 0;

unsigned long blinkTimer = 0;
const unsigned long blinkInterval = 500; // 1s blink for edit brackets
bool blinkState = true;

// auto-refresh main screen every 20s
static unsigned long lastMainUpdateMillis = 0;
static const unsigned long MAIN_UPDATE_MS = 20000; // 20 seconds

bool editingPass = false;
char inputBuffer[64] = {0};
Schedule tempSchedule;
MenuState menuState = MAIN_SCREEN;
// temporary edit value used when editing thresholds so remote updates don't overwrite in-progress edits
static float tempEditVal = 0.0f;
void initButtons() {
  pinMode(BTN_OK, INPUT_PULLUP);
  pinMode(BTN_BACK, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
}

void handleLongLeft();  // forward declaration

// LCD last-line buffer for delta rendering
static char lcd_last[4][21];

static void lcdInitBuffer() {
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 20; c++) lcd_last[r][c] = ' ';
    lcd_last[r][20] = '\0';
  }
}

// Backlight / lifetime management
static unsigned long lastActivityMillis = 0;
static bool backlightOn = true;
static const unsigned long BACKLIGHT_IDLE_MS = 120000; // 2 minutes idle -> turn off backlight

// Initialize LCD helper: buffer + backlight state
void initLCD() {
  lcdInitBuffer();
  lastActivityMillis = millis();
  backlightOn = true;
}

// Record a user activity (button press) so auto-backlight timer resets
static void recordUserActivity() {
  lastActivityMillis = millis();
  if (!backlightOn) {
    if (lcdMutex && xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      lcd.backlight();
      xSemaphoreGive(lcdMutex);
    }
    backlightOn = true;
  }
}

// Write a full 20-char line if it differs from last
static void lcdWriteLineIfChanged(int row, const char* text) {
  char desired[21];
  // build desired line padded to 20
  size_t len = strlen(text);
  if (len > 20) len = 20;
  memcpy(desired, text, len);
  for (size_t i = len; i < 20; i++) desired[i] = ' ';
  desired[20] = '\0';

  if (memcmp(desired, lcd_last[row], 20) != 0) {
    if (lcdMutex && xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      // NOTE: do NOT automatically turn on backlight for background/automatic
      // updates (to avoid waking the display and causing flicker). Only
      // explicit user activity should call `recordUserActivity()` which will
      // turn the backlight on.
      lcd.setCursor(0, row);
      lcd.print(desired);
      xSemaphoreGive(lcdMutex);
      memcpy(lcd_last[row], desired, 21);
    }
  }
}

void handleButtons() {
  feedWatchdog();
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;
  const unsigned long longPressDelay = 800;

  const uint8_t buttons[6] = {BTN_OK, BTN_BACK, BTN_LEFT, BTN_RIGHT, BTN_UP, BTN_DOWN};
  static bool lastState[6] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
  static unsigned long downTime[6] = {0};

  unsigned long currentTime = millis();

  if (currentTime - lastDebounceTime > debounceDelay) {
    lastDebounceTime = currentTime;
    for (int i = 0; i < 6; i++) {
      bool currentState = digitalRead(buttons[i]);
      if (lastState[i] == HIGH && currentState == LOW) downTime[i] = currentTime;
      if (lastState[i] == LOW && currentState == HIGH) {
        unsigned long pressDuration = currentTime - downTime[i];
        if (pressDuration >= longPressDelay && i == 2) handleLongLeft();
        else if (pressDuration > 50) {
          // user pressed a button -> record activity
          recordUserActivity();
          switch (i) {
            case 0: handleOK(); break;
            case 1: handleBack(); break;
            case 2: handleLeft(); break;
            case 3: handleRight(); break;
            case 4: handleUp(); break;
            case 5: handleDown(); break;
          }
        }
        downTime[i] = 0;
      }
      lastState[i] = currentState;
    }
  }
}

void handleLongLeft() {
  if (menuState == SCHEDULE_MENU && subIndex < settings.numSchedules) {
    for (int i = subIndex; i < settings.numSchedules - 1; i++) {
      settings.schedules[i] = settings.schedules[i + 1];
    }
    settings.numSchedules--;
    saveSettings();
    if (subIndex >= settings.numSchedules && settings.numSchedules > 0) subIndex = settings.numSchedules - 1;
    drawScheduleMenu();
  }
}

void drawMainScreen() {
  feedWatchdog();
  if (dhtFailCount > 5) {
    lcdWriteLineIfChanged(0, "DHT ERROR       ");
    lcdWriteLineIfChanged(1, "Check sensor    ");
    return;
  }
  char line0[21];
  char line1[21];
  char line2[21];
  char line3[21];
  snprintf(line0, sizeof(line0), "T:%4.1fC H:%02d%% pH:%3.1f", state.temp, (int)state.hum, state.ph);
  snprintf(line1, sizeof(line1), "S1:%3d%% S2:%3d%%", state.soil1, state.soil2);
  snprintf(line2, sizeof(line2), "Light:%3d%% WiFi:%s", state.light, WiFi.status()==WL_CONNECTED?"ON":"OFF");
  snprintf(line3, sizeof(line3), "P:%s F:%s L:%s", state.pump?"ON":"OFF", state.fan?"ON":"OFF", state.lightOn?"ON":"OFF");
  lcdWriteLineIfChanged(0, line0);
  lcdWriteLineIfChanged(1, line1);
  lcdWriteLineIfChanged(2, line2);
  lcdWriteLineIfChanged(3, line3);
}

void drawMainMenu() {
  // build 4 lines of menu text and write if changed
  const char* items[] = {
    "Lich tuoi ",
    "Nguong    ",
    "Control   ",
    "Version   ",
    "Info      ",
    "Manual    ",
    "WiFi      "
  };
  char line[4][21];
  for (int r = 0; r < 4; r++) {
    int leftIdx = r * 2;
    int rightIdx = r * 2 + 1;
    char tmp[21];
    // always show selection marker '>' so user can see current focus
    snprintf(tmp, sizeof(tmp), "%s%s", (leftIdx < 7 && menuIndex == leftIdx) ? ">" : " ", (leftIdx < 7) ? items[leftIdx] : "                    ");
    // right part
    char rightPart[11] = "          ";
    if (rightIdx < 7) {
      snprintf(rightPart, sizeof(rightPart), "%s%s", (menuIndex == rightIdx) ? ">" : " ", items[rightIdx]);
    }
    snprintf(line[r], sizeof(line[r]), "%s%s", tmp, rightPart);
    lcdWriteLineIfChanged(r, line[r]);
  }
}

void drawAutoControlMenu() {
  char l0[21], l1[21], l2[21];
  snprintf(l0, sizeof(l0), "%sPump Auto: %s", (menuIndex == 0 && blinkState) ? ">" : " ", settings.pumpAuto ? "ON" : "OFF");
  snprintf(l1, sizeof(l1), "%sFan  Auto: %s", (menuIndex == 1 && blinkState) ? ">" : " ", settings.fanAuto ? "ON" : "OFF");
  snprintf(l2, sizeof(l2), "%sLightAuto: %s", (menuIndex == 2 && blinkState) ? ">" : " ", settings.lightAuto ? "ON" : "OFF");
  // ensure selection marker always visible
  snprintf(l0, sizeof(l0), "%sPump Auto: %s", (menuIndex == 0) ? ">" : " ", settings.pumpAuto ? "ON" : "OFF");
  snprintf(l1, sizeof(l1), "%sFan  Auto: %s", (menuIndex == 1) ? ">" : " ", settings.fanAuto ? "ON" : "OFF");
  snprintf(l2, sizeof(l2), "%sLightAuto: %s", (menuIndex == 2) ? ">" : " ", settings.lightAuto ? "ON" : "OFF");
  lcdWriteLineIfChanged(0, l0);
  lcdWriteLineIfChanged(1, l1);
  lcdWriteLineIfChanged(2, l2);
  // clear bottom line to avoid leftover text from previous screens
  lcdWriteLineIfChanged(3, "                    ");
}

void drawScheduleMenu() {
  char line0[21];
  snprintf(line0, sizeof(line0), "Schedul: %d", settings.numSchedules);
  lcdWriteLineIfChanged(0, line0);
  // bottom symbols + and - on line 3
  char bottom[21];
  for (int i = 0; i < 20; i++) bottom[i] = ' ';
  bottom[20] = '\0';
  if (settings.numSchedules < MAX_SCHEDULES) bottom[18] = '+';
  bottom[19] = '-';
  lcdWriteLineIfChanged(3, bottom);

  for (int i = 0; i < 3; i++) {
    int idx = subIndex + i;
    if (idx > settings.numSchedules) break;
    char ln[21];
    if (idx == settings.numSchedules) {
      // always show marker for selected add-new line
      snprintf(ln, sizeof(ln), "%s Add new    ", (idx == subIndex) ? ">" : " ");
    } else {
      char t[8];
      snprintf(t, sizeof(t), "%02d:%02d", settings.schedules[idx].hour, settings.schedules[idx].minute);
      // show '>' for selected row instead of blinking bracket
      snprintf(ln, sizeof(ln), "%s%s   %s%s", (idx == subIndex) ? ">" : " ", t,
               settings.schedules[idx].forPump ? "P" : " ", settings.schedules[idx].forLight ? "L" : " ");
    }
    lcdWriteLineIfChanged(i+1, ln);
  }
}

void drawEditSchedule() {
  char l0[21], l1[21], l2[21], l3[21];
  snprintf(l0, sizeof(l0), "Edit Schedule     ");
  snprintf(l1, sizeof(l1), "Time: %s%02d%s:%s%02d%s",
           (editIndex == 0 && blinkState) ? "[" : " ", tempSchedule.hour, (editIndex == 0 && blinkState) ? "]" : " ",
           (editIndex == 1 && blinkState) ? "[" : " ", tempSchedule.minute, (editIndex == 1 && blinkState) ? "]" : " ");
  snprintf(l2, sizeof(l2), "Pump : %s", (editIndex == 2 && blinkState) ? (tempSchedule.forPump ? "[Yes]" : "[No ]") : (tempSchedule.forPump ? "Yes " : "No  "));
  snprintf(l3, sizeof(l3), "Light: %s", (editIndex == 3 && blinkState) ? (tempSchedule.forLight ? "[Yes]" : "[No ]") : (tempSchedule.forLight ? "Yes " : "No  "));
  lcdWriteLineIfChanged(0, l0);
  lcdWriteLineIfChanged(1, l1);
  lcdWriteLineIfChanged(2, l2);
  lcdWriteLineIfChanged(3, l3);
}

void drawThresholdMenu() {
  // build 4 lines and write with delta-rendering to avoid flicker
  const char* items[] = {"Temp","Humidity","Soil","Light","pH Min","pH Max"};
  char line[4][21];
  for (int r = 0; r < 4; r++) {
    int leftIdx = r * 2;
    int rightIdx = r * 2 + 1;
    char left[11] = "          ";
    char right[11] = "          ";
    if (leftIdx < 6) snprintf(left, sizeof(left), "%s%-9s", (menuIndex==leftIdx)?">":" ", items[leftIdx]);
    if (rightIdx < 6) snprintf(right, sizeof(right), "%s%-9s", (menuIndex==rightIdx)?">":" ", items[rightIdx]);
    snprintf(line[r], sizeof(line[r]), "%s%s", left, right);
    lcdWriteLineIfChanged(r, line[r]);
  }
}

void drawEditThreshold(float currentVal, float editVal, const char* label) {
  char line0[21];
  char line1[21];
  char line2[21];
  char line3[21];
  // Line0: show current applied value (from settings/server)
  snprintf(line0, sizeof(line0), "Curr %s:%5.1f", label, currentVal);
  // blank line
  snprintf(line1, sizeof(line1), "                    ");
  // Line2: show editable value in brackets that blink
  if (blinkState) {
    snprintf(line2, sizeof(line2), "[%5.1f]              ", editVal);
  } else {
    snprintf(line2, sizeof(line2), " %5.1f               ", editVal);
  }
  snprintf(line3, sizeof(line3), "                    ");
  lcdWriteLineIfChanged(0, line0);
  lcdWriteLineIfChanged(1, line1);
  lcdWriteLineIfChanged(2, line2);
  lcdWriteLineIfChanged(3, line3);
}

void drawVersionMenu() {
  char line0[21];
  char line1[21];
  char line2[21];
  char line3[21];
  snprintf(line0, sizeof(line0), "Firmware Ver      ");
  snprintf(line1, sizeof(line1), "%s", FIRMWARE_VERSION);
  snprintf(line2, sizeof(line2), "                    ");
  snprintf(line3, sizeof(line3), "%s", blinkState ? "> Check Update" : "  Check Update");
  lcdWriteLineIfChanged(0, line0);
  lcdWriteLineIfChanged(1, line1);
  lcdWriteLineIfChanged(2, line2);
  lcdWriteLineIfChanged(3, line3);
}

void drawInfoMenu() {
  char line0[21];
  char line1[21];
  char line2[21];
  char line3[21];
  snprintf(line0, sizeof(line0), "Device ID: %s", settings.deviceID);
  snprintf(line1, sizeof(line1), "Token: %s", settings.token);
  // explicitly clear line2 to avoid leftover text from previous menus
  snprintf(line2, sizeof(line2), "                    ");
  snprintf(line3, sizeof(line3), "%s", settings.addedToWeb ? "Added: Yes" : "Added: No ");
  lcdWriteLineIfChanged(0, line0);
  lcdWriteLineIfChanged(1, line1);
  lcdWriteLineIfChanged(2, line2);
  lcdWriteLineIfChanged(3, line3);
}

void drawManualControl() {
  // build 3 lines to avoid direct lcd.clear()/print which cause race conditions
  const char* labels[] = {"Pump","Fan","Light"};
  char ln[4][21];
  for (int i = 0; i < 3; i++) {
    // If auto-mode for this relay is enabled, show AUTO and disable manual toggle
    const char* status;
    if (i == 0 && settings.pumpAuto) status = "AUTO";
    else if (i == 1 && settings.fanAuto) status = "AUTO";
    else if (i == 2 && settings.lightAuto) status = "AUTO";
    else status = (i==0? (state.pump?"ON":"OFF") : (i==1? (state.fan?"ON":"OFF") : (state.lightOn?"ON":"OFF")));
    snprintf(ln[i], sizeof(ln[i]), "%s%-7s: %s", (menuIndex==i)?">":" ", labels[i], status);
    lcdWriteLineIfChanged(i, ln[i]);
  }
  // ensure bottom line cleared to avoid leftover characters
  lcdWriteLineIfChanged(3, "                    ");
}

void drawWiFiSetup() {
  char line0[21];
  char line1[21];
  snprintf(line0, sizeof(line0), "%s", editingPass ? "Password:" : "SSID:");
  // ensure inputBuffer is null-terminated
  inputBuffer[sizeof(inputBuffer)-1] = '\0';
  snprintf(line1, sizeof(line1), "%s", inputBuffer);
  // indicate cursor position by replacing char at inputPos with '_' if in bounds
  size_t len = strlen(line1);
  if (inputPos < 20) {
    char tmp[21];
    strncpy(tmp, line1, 20);
    tmp[20] = '\0';
    if (inputPos < strlen(inputBuffer)) tmp[inputPos] = '_';
    else tmp[len < 20 ? len : 19] = '_';
    lcdWriteLineIfChanged(1, tmp);
  } else {
    lcdWriteLineIfChanged(1, line1);
  }
  lcdWriteLineIfChanged(0, line0);
  // clear remaining lines to avoid leftover content
  lcdWriteLineIfChanged(2, "                    ");
  lcdWriteLineIfChanged(3, "                    ");
}

void handleOK() {
  switch (menuState) {
    case MAIN_SCREEN: {
      static int okCount = 0;
      static unsigned long firstOkTs = 0;
      unsigned long now = millis();
      if (firstOkTs == 0 || now - firstOkTs > 2000) { firstOkTs = now; okCount = 1; }
      else okCount++;
      if (okCount >= 3) {
        okCount = 0; firstOkTs = 0;
        // triple-OK pressed: if deepSleep enabled, enter deep sleep
        if (settings.deepSleep) {
          // persist and notify
          saveSettingsNow();
          requestTelemetrySend();
          delay(300);
          // configure wake on OK (active LOW) and deep sleep
          esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_OK, 0);
          Serial.println("Entering deep sleep via triple-OK");
          ESP.deepSleep(0);
        }
        // otherwise open menu
        menuState = MAIN_MENU; menuIndex = 0; drawMainMenu();
      } else {
        // short press: open main menu
        menuState = MAIN_MENU; menuIndex = 0; drawMainMenu();
      }
    }
    break;
    case MAIN_MENU:
      switch (menuIndex) {
        case 0: menuState = SCHEDULE_MENU; subIndex = 0; drawScheduleMenu(); break;
        case 1: menuState = THRESHOLD_MENU; menuIndex = 0; drawThresholdMenu(); break;
        case 2: menuState = AUTO_CONTROL_MENU; menuIndex = 0; drawAutoControlMenu(); break;
        case 3: menuState = VERSION_MENU; drawVersionMenu(); break;
        case 4: menuState = INFO_MENU; drawInfoMenu(); break;
        case 5: menuState = MANUAL_CONTROL; menuIndex = 0; drawManualControl(); break;
        case 6: menuState = WIFI_SETUP; editingPass = false; strncpy(inputBuffer, settings.ssid, sizeof(inputBuffer)-1); inputBuffer[sizeof(inputBuffer)-1] = '\0'; inputPos = strlen(inputBuffer); drawWiFiSetup(); break;
      }
      break;
    case SCHEDULE_MENU:
      if (subIndex == settings.numSchedules) {
        tempSchedule = {0, 0, true, false};
      } else {
        tempSchedule = settings.schedules[subIndex];
      }
      editIndex = 0;
      menuState = EDIT_SCHEDULE;
      drawEditSchedule();
      break;
    case EDIT_SCHEDULE:
      if (editIndex < 3) editIndex++;
      else {
        if (subIndex < settings.numSchedules) {
          settings.schedules[subIndex] = tempSchedule;
        } else if (settings.numSchedules < MAX_SCHEDULES) {
          settings.schedules[settings.numSchedules++] = tempSchedule;
        }
          saveSettingsNow();
          // suppress remote updates longer so backend has time to persist
          suppressRemoteUntil = millis() + 30000;
          requestTelemetrySendPersist();
        menuState = MAIN_SCREEN;
        drawMainScreen();
      }
      drawEditSchedule();
      break;
    
    case AUTO_CONTROL_MENU:
      if (menuIndex == 0) { settings.pumpAuto = !settings.pumpAuto; if (settings.pumpAuto) settings.relayOverride = false; }
      if (menuIndex == 1) { settings.fanAuto = !settings.fanAuto; if (settings.fanAuto) settings.relayOverride = false; }
      if (menuIndex == 2) { settings.lightAuto = !settings.lightAuto; if (settings.lightAuto) settings.relayOverride = false; }
      // Persist immediately and notify backend so the device state is authoritative
      saveSettingsNow();
      // suppress remote updates longer so backend has time to persist
      suppressRemoteUntil = millis() + 30000;
      requestTelemetrySendPersist();
      drawAutoControlMenu();
      break;

    case THRESHOLD_MENU:
      switch (menuIndex) {
        case 0: menuState = EDIT_TEMP; tempEditVal = settings.tempThresh; drawEditThreshold(settings.tempThresh, tempEditVal, "Temp"); break;
        case 1: menuState = EDIT_HUM; tempEditVal = settings.humThresh; drawEditThreshold(settings.humThresh, tempEditVal, "Hum"); break;
        case 2: menuState = EDIT_SOIL; tempEditVal = settings.soilThresh; drawEditThreshold(settings.soilThresh, tempEditVal, "Soil"); break;
        case 3: menuState = EDIT_LIGHT; tempEditVal = settings.lightThresh; drawEditThreshold(settings.lightThresh, tempEditVal, "Light"); break;
        case 4: menuState = EDIT_PH_MIN; tempEditVal = settings.phThreshMin; drawEditThreshold(settings.phThreshMin, tempEditVal, "pH Min"); break;
        case 5: menuState = EDIT_PH_MAX; tempEditVal = settings.phThreshMax; drawEditThreshold(settings.phThreshMax, tempEditVal, "pH Max"); break;
      }
      break;
    case EDIT_TEMP: case EDIT_HUM: case EDIT_SOIL: case EDIT_LIGHT: case EDIT_PH_MIN: case EDIT_PH_MAX:
      // commit temporary edit value into settings and persist immediately
      if (menuState == EDIT_TEMP) settings.tempThresh = tempEditVal;
      else if (menuState == EDIT_HUM) settings.humThresh = tempEditVal;
      else if (menuState == EDIT_SOIL) settings.soilThresh = tempEditVal;
      else if (menuState == EDIT_LIGHT) settings.lightThresh = tempEditVal;
      else if (menuState == EDIT_PH_MIN) settings.phThreshMin = tempEditVal;
      else if (menuState == EDIT_PH_MAX) settings.phThreshMax = tempEditVal;
      saveSettingsNow();
      // suppress remote updates longer to allow server to persist
      suppressRemoteUntil = millis() + 30000;
      requestTelemetrySendPersist();
      menuState = MAIN_SCREEN;
      drawMainScreen();
      break;
    case LIGHT_SET_MENU:
      settings.lightAuto = !settings.lightAuto;
      if (settings.lightAuto) settings.relayOverride = false;
      saveSettingsNow();
      suppressRemoteUntil = millis() + 30000;
      requestTelemetrySendPersist();
      menuState = MAIN_SCREEN;
      drawMainScreen();
      break;
    case VERSION_MENU:
      lcdWriteLineIfChanged(0, "Checking update     ");
      requestOTA();
      delay(1000);
      menuState = MAIN_SCREEN;
      drawMainScreen();
      break;
      break;
    case MANUAL_CONTROL: {
      bool ignored = false;
      if (menuIndex == 0) {
        if (settings.pumpAuto) ignored = true;
        else {
          state.pump = !state.pump; digitalWrite(RELAY_PUMP, state.pump);
          settings.pumpAuto = false; settings.relayOverride = true;
          saveSettingsNow(); suppressRemoteUntil = millis() + 30000; requestTelemetrySendPersist();
        }
      }
      if (menuIndex == 1) {
        if (settings.fanAuto) ignored = true;
        else {
          state.fan = !state.fan; digitalWrite(RELAY_FAN, state.fan);
          settings.fanAuto = false; settings.relayOverride = true;
          saveSettingsNow(); suppressRemoteUntil = millis() + 30000; requestTelemetrySendPersist();
        }
      }
      if (menuIndex == 2) {
        if (settings.lightAuto) ignored = true;
        else {
          state.lightOn = !state.lightOn; digitalWrite(RELAY_LIGHT, state.lightOn);
          settings.lightAuto = false; settings.relayOverride = true;
          saveSettingsNow(); suppressRemoteUntil = millis() + 30000; requestTelemetrySendPersist();
        }
      }
      if (ignored) {
        lcdWriteLineIfChanged(3, "Manual disabled: AUTO");
        vTaskDelay(pdMS_TO_TICKS(800));
      }
      drawManualControl();
      break; }
    case WIFI_SETUP:
      if (!editingPass) {
        strncpy(settings.ssid, inputBuffer, sizeof(settings.ssid)-1);
        settings.ssid[sizeof(settings.ssid)-1] = '\0';
        strncpy(inputBuffer, settings.pass, sizeof(inputBuffer)-1);
        inputBuffer[sizeof(inputBuffer)-1] = '\0';
        inputPos = strlen(inputBuffer);
        editingPass = true;
      } else {
        strncpy(settings.pass, inputBuffer, sizeof(settings.pass)-1);
        settings.pass[sizeof(settings.pass)-1] = '\0';
        saveSettings();
        connectWiFi();
        menuState = MAIN_SCREEN;
        drawMainScreen();
      }
      drawWiFiSetup();
      break;
  }
}

void handleBack() {
  if (menuState == MAIN_MENU || menuState == SCHEDULE_MENU || menuState == THRESHOLD_MENU ||
      menuState == LIGHT_SET_MENU || menuState == VERSION_MENU || menuState == INFO_MENU ||
      menuState == MANUAL_CONTROL || menuState == WIFI_SETUP) {
    menuState = MAIN_SCREEN;
    drawMainScreen();
  } else if (menuState == EDIT_SCHEDULE) {
    menuState = SCHEDULE_MENU;
    drawScheduleMenu();
  } else if (menuState >= EDIT_TEMP && menuState <= EDIT_PH_MAX) {
    menuState = THRESHOLD_MENU;
    drawThresholdMenu();
  } else {
    menuState = MAIN_SCREEN;
    drawMainScreen();
  }
}

void handleUp() {
  switch (menuState) {
    case MAIN_MENU:
      if (menuIndex >= 2) menuIndex -= 2;
      else menuIndex = (menuIndex % 2 == 0) ? 6 : 5;
      drawMainMenu(); break;
    case THRESHOLD_MENU:
      if (menuIndex >= 2) menuIndex -= 2;
      else menuIndex = (menuIndex % 2 == 0) ? 4 : 5;
      drawThresholdMenu(); break;
    case SCHEDULE_MENU:
      subIndex = (subIndex == 0) ? settings.numSchedules : subIndex - 1;
      drawScheduleMenu(); break;
    case MANUAL_CONTROL:
      menuIndex = (menuIndex == 0) ? 2 : menuIndex - 1;
      drawManualControl(); break;
    case AUTO_CONTROL_MENU:  // Thêm case này để lên giảm menuIndex
      menuIndex = (menuIndex == 0) ? 2 : menuIndex - 1;
      drawAutoControlMenu(); break;
    case EDIT_SCHEDULE:
      if (editIndex == 0) tempSchedule.hour = (tempSchedule.hour + 1) % 24;
      else if (editIndex == 1) tempSchedule.minute = (tempSchedule.minute + 1) % 60;
      else if (editIndex == 2) tempSchedule.forPump = !tempSchedule.forPump;
      else if (editIndex == 3) tempSchedule.forLight = !tempSchedule.forLight;
      drawEditSchedule(); break;
    case EDIT_TEMP: tempEditVal += 0.5; drawEditThreshold(settings.tempThresh, tempEditVal, "Temp"); break;
    case EDIT_HUM: tempEditVal += 1.0; drawEditThreshold(settings.humThresh, tempEditVal, "Hum"); break;
    case EDIT_SOIL: tempEditVal += 1.0; drawEditThreshold(settings.soilThresh, tempEditVal, "Soil"); break;
    case EDIT_LIGHT: tempEditVal += 10.0; drawEditThreshold(settings.lightThresh, tempEditVal, "Light"); break;
    case EDIT_PH_MIN: tempEditVal += 0.1; drawEditThreshold(settings.phThreshMin, tempEditVal, "pH Min"); break;
    case EDIT_PH_MAX: tempEditVal += 0.1; drawEditThreshold(settings.phThreshMax, tempEditVal, "pH Max"); break;
    case WIFI_SETUP:
      {
      size_t iblen = strlen(inputBuffer);
      if (iblen == 0) inputPos = 0;
      char &c = inputBuffer[inputPos];
      if (c == ' ') c = 'z';
      else if (c == '9') c = ' ';
      else if (c == 'z') c = 'a';
      else if (c == '0') c = '9';
      else c++;
      drawWiFiSetup(); }
      break;
  }
}

void handleDown() {
  switch (menuState) {
    case MAIN_MENU:
      if (menuIndex <= 4) menuIndex += 2;
      else menuIndex = (menuIndex % 2 == 0) ? 0 : 1;
      drawMainMenu(); break;
    case THRESHOLD_MENU:
      if (menuIndex <= 3) menuIndex += 2;
      else menuIndex = (menuIndex % 2 == 0) ? 0 : 1;
      drawThresholdMenu(); break;
    case SCHEDULE_MENU:
      subIndex = (subIndex + 1) > settings.numSchedules ? 0 : subIndex + 1;
      drawScheduleMenu(); break;
    case MANUAL_CONTROL:
      menuIndex = (menuIndex + 1) % 3;
      drawManualControl(); break;
    case AUTO_CONTROL_MENU:
      menuIndex = (menuIndex + 1) % 3;
      drawAutoControlMenu(); break;
    case EDIT_SCHEDULE:
      if (editIndex == 0) tempSchedule.hour = (tempSchedule.hour == 0) ? 23 : tempSchedule.hour - 1;
      else if (editIndex == 1) tempSchedule.minute = (tempSchedule.minute == 0) ? 59 : tempSchedule.minute - 1;
      else if (editIndex == 2) tempSchedule.forPump = !tempSchedule.forPump;
      else if (editIndex == 3) tempSchedule.forLight = !tempSchedule.forLight;
      drawEditSchedule(); break;
    case EDIT_TEMP: tempEditVal -= 0.5; drawEditThreshold(settings.tempThresh, tempEditVal, "Temp"); break;
    case EDIT_HUM: tempEditVal -= 1.0; drawEditThreshold(settings.humThresh, tempEditVal, "Hum"); break;
    case EDIT_SOIL: tempEditVal -= 1.0; drawEditThreshold(settings.soilThresh, tempEditVal, "Soil"); break;
    case EDIT_LIGHT: tempEditVal -= 10.0; drawEditThreshold(settings.lightThresh, tempEditVal, "Light"); break;
    case EDIT_PH_MIN: tempEditVal -= 0.1; drawEditThreshold(settings.phThreshMin, tempEditVal, "pH Min"); break;
    case EDIT_PH_MAX: tempEditVal -= 0.1; drawEditThreshold(settings.phThreshMax, tempEditVal, "pH Max"); break;
    case WIFI_SETUP:
      {
      size_t iblen = strlen(inputBuffer);
      if (iblen == 0) inputPos = 0;
      char &c = inputBuffer[inputPos];
      if (c == ' ') c = 'a';
      else if (c == 'a') c = ' ';
      else if (c == '0') c = ' ';
      else if (c == '9') c = '0';
      else c--;
      drawWiFiSetup(); }
      break;
  }
}

void handleLeft() {
  switch (menuState) {
    case MAIN_MENU:
      if (menuIndex % 2 == 1) menuIndex--;
      else if (menuIndex >= 2) menuIndex -= 2;
      drawMainMenu(); break;
    case THRESHOLD_MENU:
      if (menuIndex % 2 == 1) menuIndex--;
      else if (menuIndex >= 2) menuIndex -= 2;
      drawThresholdMenu(); break;
    case EDIT_SCHEDULE:
      editIndex = (editIndex == 0) ? 3 : editIndex - 1;
      drawEditSchedule(); break;
    case WIFI_SETUP:
      {
        size_t iblen = strlen(inputBuffer);
        if (iblen == 0) inputPos = 0;
        else inputPos = (inputPos == 0) ? (iblen - 1) : inputPos - 1;
        drawWiFiSetup();
      }
      break;
  }
}

void handleRight() {
  switch (menuState) {
    case MAIN_MENU:
      if (menuIndex % 2 == 0 && menuIndex < 6) menuIndex++;
      else if (menuIndex <= 4) menuIndex += 2;
      drawMainMenu(); break;
    case THRESHOLD_MENU:
      if (menuIndex % 2 == 0 && menuIndex < 5) menuIndex++;
      else if (menuIndex <= 3) menuIndex += 2;
      drawThresholdMenu(); break;
    case EDIT_SCHEDULE:
      editIndex = (editIndex + 1) % 4;
      drawEditSchedule(); break;
    case WIFI_SETUP:
      {
        size_t iblen = strlen(inputBuffer);
        if (iblen == 0) inputPos = 0;
        else inputPos = (inputPos + 1) % iblen;
        drawWiFiSetup();
      }
      break;
  }
}

void updateMenu() {
  feedWatchdog();

  unsigned long now = millis();
  bool blinkChanged = false;
  if (now - blinkTimer >= blinkInterval) {
    blinkState = !blinkState;
    blinkTimer = now;
    blinkChanged = true;
  }

  bool needFullRedraw = (menuState != lastState) ||
                        (menuIndex != lastMenuIndex) ||
                        (menuState == SCHEDULE_MENU && subIndex != lastSubIndex);

  if (needFullRedraw) {
    // Redraw toàn bộ khi chuyển menu hoặc chọn mục mới
    switch (menuState) {
      case MAIN_SCREEN: drawMainScreen(); break;
      case MAIN_MENU: drawMainMenu(); break;
      case SCHEDULE_MENU: drawScheduleMenu(); break;
      case EDIT_SCHEDULE: drawEditSchedule(); break;
      case THRESHOLD_MENU: drawThresholdMenu(); break;
      case AUTO_CONTROL_MENU: drawAutoControlMenu(); break;
      case VERSION_MENU: drawVersionMenu(); break;
      case INFO_MENU: drawInfoMenu(); break;
      case MANUAL_CONTROL: drawManualControl(); break;
      case WIFI_SETUP: drawWiFiSetup(); break;
      case EDIT_TEMP: case EDIT_HUM: case EDIT_SOIL:
      case EDIT_LIGHT: case EDIT_PH_MIN: case EDIT_PH_MAX:
        // sẽ redraw trong handleUp/Down/OK
        break;
    }
    lastState = menuState;
    lastMenuIndex = menuIndex;
    lastSubIndex = (menuState == SCHEDULE_MENU) ? subIndex : lastSubIndex;
  } 
  else if (blinkChanged) {
    // Chỉ redraw các menu cần nháy con trỏ
    switch (menuState) {
      case MAIN_MENU: drawMainMenu(); break;
      case THRESHOLD_MENU: drawThresholdMenu(); break;
      case SCHEDULE_MENU: drawScheduleMenu(); break;
      case EDIT_SCHEDULE: drawEditSchedule(); break;
      case AUTO_CONTROL_MENU: drawAutoControlMenu(); break;
      case VERSION_MENU: drawVersionMenu(); break;
      case MANUAL_CONTROL: drawManualControl(); break;
      case EDIT_TEMP: case EDIT_HUM: case EDIT_SOIL:
      case EDIT_LIGHT: case EDIT_PH_MIN: case EDIT_PH_MAX:
        // redraw giá trị đang edit: pass both current (settings) and in-progress edit value
        if (menuState == EDIT_TEMP) drawEditThreshold(settings.tempThresh, tempEditVal, "Temp");
        else if (menuState == EDIT_HUM) drawEditThreshold(settings.humThresh, tempEditVal, "Hum");
        else if (menuState == EDIT_SOIL) drawEditThreshold(settings.soilThresh, tempEditVal, "Soil");
        else if (menuState == EDIT_LIGHT) drawEditThreshold(settings.lightThresh, tempEditVal, "Light");
        else if (menuState == EDIT_PH_MIN) drawEditThreshold(settings.phThreshMin, tempEditVal, "pH Min");
        else if (menuState == EDIT_PH_MAX) drawEditThreshold(settings.phThreshMax, tempEditVal, "pH Max");
        break;
        break;
    }
  }
  // Periodic main-screen refresh (update sensor values without user input)
  if (menuState == MAIN_SCREEN) {
    if (now - lastMainUpdateMillis >= MAIN_UPDATE_MS || needMainRefresh) {
      drawMainScreen();
      lastMainUpdateMillis = now;
      needMainRefresh = false;
    }
  }
}

// UI RTOS primitives
typedef struct {
  uint8_t id; // 0=OK,1=BACK,2=LEFT,3=RIGHT,4=UP,5=DOWN
  bool longPress;
  uint32_t ts;
} ButtonEvent;

static QueueHandle_t buttonQueue = NULL;
// LCD message queue for other tasks to request line updates
typedef struct {
  uint8_t row;
  char text[21];
} LCDMessage;
static QueueHandle_t lcdQueue = NULL;

void lcdPostLine(int row, const char* text) {
  // Only allow background tasks to post lines when main screen is active
  if (menuState != MAIN_SCREEN) return;
  if (!lcdQueue) return;
  LCDMessage msg;
  msg.row = row;
  strncpy(msg.text, text, 20);
  msg.text[20] = '\0';
  xQueueSend(lcdQueue, &msg, 0);
}

// Button polling task: lightweight, only enqueues events
void buttonTask(void *param) {
  const uint8_t buttons[6] = {BTN_OK, BTN_BACK, BTN_LEFT, BTN_RIGHT, BTN_UP, BTN_DOWN};
  static bool lastState[6] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
  static unsigned long downTime[6] = {0};
  const unsigned long debounceDelay = 50;
  const unsigned long longPressDelay = 800;

  unsigned long lastDebounce = 0;
  while (1) {
    feedWatchdog();
    unsigned long now = millis();
    if (now - lastDebounce > 10) { // poll every ~10ms
      lastDebounce = now;
      for (int i = 0; i < 6; i++) {
        bool current = digitalRead(buttons[i]);
        if (lastState[i] == HIGH && current == LOW) {
          downTime[i] = now;
        }
        if (lastState[i] == LOW && current == HIGH) {
          unsigned long pressDuration = now - downTime[i];
          if (pressDuration >= longPressDelay && i == 2) {
            // special long-left action
            ButtonEvent ev = {(uint8_t)i, true, now};
            if (buttonQueue) xQueueSend(buttonQueue, &ev, 0);
          } else if (pressDuration > debounceDelay) {
            ButtonEvent ev = {(uint8_t)i, false, now};
            if (buttonQueue) xQueueSend(buttonQueue, &ev, 0);
          }
          downTime[i] = 0;
        }
        lastState[i] = current;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// LCD/render task: owns all LCD operations and calls menu handlers
void lcdTask(void *param) {
  const TickType_t tick = pdMS_TO_TICKS(250);
  ButtonEvent ev;
  while (1) {
    feedWatchdog();
    // process incoming LCD messages (from other tasks)
    if (lcdQueue) {
      LCDMessage lm;
      while (xQueueReceive(lcdQueue, &lm, 0) == pdPASS) {
        lcdWriteLineIfChanged(lm.row, lm.text);
      }
    }
    // process incoming button events
    while (buttonQueue && xQueueReceive(buttonQueue, &ev, 0) == pdPASS) {
        // record user activity for auto-backlight reset
        recordUserActivity();
        // map id to handlers (called in lcdTask so LCD ops safe)
        switch (ev.id) {
          case 0: handleOK(); break;
          case 1: handleBack(); break;
          case 2: if (ev.longPress) handleLongLeft(); else handleLeft(); break;
          case 3: handleRight(); break;
          case 4: handleUp(); break;
          case 5: handleDown(); break;
        }
    }

    // periodic redraw / blink handling (handled inside updateMenu)
    updateMenu();

    // auto-backlight: if no activity for BACKLIGHT_IDLE_MS, turn off backlight
    if (backlightOn && (millis() - lastActivityMillis > BACKLIGHT_IDLE_MS)) {
      if (lcdMutex && xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        lcd.noBacklight();
        xSemaphoreGive(lcdMutex);
      }
      backlightOn = false;
    }

    vTaskDelay(tick);
  }
}

void startUITasks() {
  if (buttonQueue == NULL) buttonQueue = xQueueCreate(10, sizeof(ButtonEvent));
  if (lcdQueue == NULL) lcdQueue = xQueueCreate(10, sizeof(LCDMessage));
  // initialize user activity timer so backlight doesn't immediately turn off
  lastActivityMillis = millis();
  backlightOn = true;
  // create tasks pinned to core 1
  xTaskCreatePinnedToCore(buttonTask, "ButtonTask", 4096, NULL, 3, &buttonTaskHandle, 1);
  xTaskCreatePinnedToCore(lcdTask, "LCDTask", 8192, NULL, 2, &lcdTaskHandle, 1);
}
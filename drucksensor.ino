#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

#define USE_PT100 false

#if USE_PT100
#include <Adafruit_MAX31865.h>
#endif

// ===================== PINS =====================
const int PIN_ENCODER_CLK = 2;
const int PIN_ENCODER_DT  = 3;
const int PIN_ENCODER_SW  = 4;
const int PIN_TEMP        = 5;   // reserviert / optional
const int PIN_BUZZER      = 6;
const int PIN_OPTO        = 7;

const int PIN_PRESSURE    = A0;

// ===================== DISPLAY =====================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===================== MAX31865 / PT100 =====================
#if USE_PT100
#define MAX31865_CS 10
Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS);
#define RREF      430.0
#define RNOMINAL  100.0
#endif

enum TempStatus {
  TEMP_NO_MAX,
  TEMP_OK,
  TEMP_SENSOR_ERR
};

TempStatus tempStatus = TEMP_NO_MAX;
float currentTemp = 0.0f;

// ===================== DRUCKSTATUS =====================
enum PressureStatus {
  PRESS_NO_SENSOR,
  PRESS_OK,
  PRESS_SENSOR_ERR
};

PressureStatus pressureStatus = PRESS_NO_SENSOR;

// ===================== ENCODER =====================
long encoderPos = 0;
int lastCLKState;
bool lastButtonState = HIGH;
unsigned long lastButtonTime = 0;

// ===================== RPM =====================
bool lastOptoState = LOW;
unsigned long lastPulseMicros = 0;
float currentRPM = 0.0f;

byte magnetsPerRev = 1;

const unsigned long MIN_PULSE_US = 1000UL;
const unsigned long MAX_PULSE_US = 2000000UL;

const byte NUM_PERIOD_SAMPLES = 3;
unsigned long periodSamples[NUM_PERIOD_SAMPLES];
byte periodIndex = 0;
bool periodBufferFilled = false;

// ===================== DRUCK =====================
float currentBar = 0.0f;
const float PRESSURE_MAX_BAR = 25.0f;
const float SHUNT_OHM = 120.0f;
const float ADC_REF_V = 5.0f;

// ===================== WARN/ALARM =====================
bool pressureWarnActive = false;
bool pressureAlarmActive = false;

const float PRESSURE_HYSTERESIS = 0.3f;

// ===================== SCHNECKEN =====================
struct ScrewProfile {
  char  name[9];
  float litersPerRev;
  float warnPressureBar;
  float alarmPressureBar;
};

const byte NUM_SCREWS = 5;
ScrewProfile screws[NUM_SCREWS] = {
  { "D4-3", 0.035f, 10.0f, 12.0f },
  { "SN2",  0.000f, 10.0f, 12.0f },
  { "SN3",  0.000f, 10.0f, 12.0f },
  { "SN4",  0.000f, 10.0f, 12.0f },
  { "SN5",  0.000f, 10.0f, 12.0f }
};

byte currentScrew = 0;
byte currentScrewCount = 1;
byte pendingScrew = 0;

// ===================== UI =====================
enum UiMode {
  UI_VIEW,
  UI_MENU,
  UI_SELECT_SCREW,
  UI_EDIT_FACTOR,
  UI_EDIT_NAME,
  UI_EDIT_MAGNETS,
  UI_EDIT_WARN_PRESSURE,
  UI_EDIT_ALARM_PRESSURE
};

UiMode uiMode = UI_VIEW;

const char* menuItems[] = {
  "Schnecke wahl",
  "Faktor edit",
  "Name edit",
  "Magnete",
  "Druckwarnung",
  "Druckalarm",
  "Schnecke +",
  "Schnecke -",
  "Zurueck"
};

const byte NUM_MENU_ITEMS = sizeof(menuItems) / sizeof(menuItems[0]);
byte menuIndex = 0;
byte nameEditPos = 0;

// ===================== HILFSFUNKTIONEN =====================
void formatFloat(float v, int width, int prec, char* out, size_t outSz) {
  char buf[16];
  dtostrf(v, width, prec, buf);
  strncpy(out, buf, outSz);
  out[outSz - 1] = '\0';
}

void printPaddedLine(uint8_t row, const char* text) {
  lcd.setCursor(0, row);
  lcd.print(text);
  int len = strlen(text);
  for (int i = len; i < 16; i++) lcd.print(' ');
}

char nextChar(char c, int dir) {
  const char charset[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_";
  const int N = sizeof(charset) - 1;
  int idx = 0;
  for (int i = 0; i < N; i++) {
    if (charset[i] == c) {
      idx = i;
      break;
    }
  }
  idx += dir;
  if (idx < 0) idx = N - 1;
  if (idx >= N) idx = 0;
  return charset[idx];
}

void intToRpmText(int rpm, char* out, size_t outSz) {
  snprintf(out, outSz, "%4drpm", rpm);
}

// ===================== ENCODER =====================
void updateEncoder() {
  static unsigned long lastEncUs = 0;
  int clkState = digitalRead(PIN_ENCODER_CLK);

  if (clkState != lastCLKState) {
    if (clkState == HIGH) {
      unsigned long now = micros();
      if (now - lastEncUs > 3000) {
        if (digitalRead(PIN_ENCODER_DT) == LOW) encoderPos++;
        else                                    encoderPos--;
        lastEncUs = now;
      }
    }
    lastCLKState = clkState;
  }
}

bool buttonPressed() {
  bool reading = digitalRead(PIN_ENCODER_SW);
  bool pressed = false;

  if (reading == LOW && lastButtonState == HIGH) {
    if (millis() - lastButtonTime > 200) {
      pressed = true;
      lastButtonTime = millis();
    }
  }
  lastButtonState = reading;
  return pressed;
}

// ===================== SENSOREN =====================
void measureRPM() {
  bool optoState = digitalRead(PIN_OPTO);

  if (optoState == HIGH && lastOptoState == LOW) {
    unsigned long now = micros();
    unsigned long diff = now - lastPulseMicros;
    lastPulseMicros = now;

    if (diff >= MIN_PULSE_US && diff <= MAX_PULSE_US) {
      periodSamples[periodIndex] = diff;
      periodIndex++;
      if (periodIndex >= NUM_PERIOD_SAMPLES) {
        periodIndex = 0;
        periodBufferFilled = true;
      }

      byte count = periodBufferFilled ? NUM_PERIOD_SAMPLES : periodIndex;
      unsigned long sum = 0;
      for (byte i = 0; i < count; i++) sum += periodSamples[i];

      float avgPeriodSec = (sum / (float)count) / 1000000.0f;

      if (avgPeriodSec > 0.0f && magnetsPerRev > 0) {
        float revPerSec = 1.0f / (avgPeriodSec * (float)magnetsPerRev);
        currentRPM = revPerSec * 60.0f;
      }
    }
  }
  lastOptoState = optoState;

  if (micros() - lastPulseMicros > 2000000UL) {
    currentRPM = 0.0f;
    periodBufferFilled = false;
    periodIndex = 0;
  }
}
void measurePressure() {
  const byte NUM_SAMPLES = 16;
  unsigned long sum = 0;

  for (byte i = 0; i < NUM_SAMPLES; i++) {
    sum += analogRead(PIN_PRESSURE);
    delayMicroseconds(300);
  }

  float raw = sum / (float)NUM_SAMPLES;
  float voltage = (raw / 1023.0f) * ADC_REF_V;
  float current_mA = (voltage / SHUNT_OHM) * 1000.0f;

  if (voltage < 0.2f) {
  pressureStatus = PRESS_NO_SENSOR;
  currentBar = 0.0f;
  return;
}

if (voltage < 0.4f) {
  pressureStatus = PRESS_SENSOR_ERR;
  currentBar = 0.0f;
  return;
}

  // ===== PLAUSIBLER BEREICH =====
  if (current_mA < 3.8f || current_mA > 21.0f) {
    pressureStatus = PRESS_SENSOR_ERR;
    currentBar = 0.0f;
    return;
  }

  // ===== BERECHNUNG =====
  float bar = ((current_mA - 4.0f) / 16.0f) * 25.0f;

  if (bar < 0.0f) bar = 0.0f;
  if (bar > 25.0f) bar = 25.0f;

  currentBar = bar;
  pressureStatus = PRESS_OK;
}

void measureTemp() {
#if USE_PT100
  uint8_t fault = thermo.readFault();

  if (fault) {
    tempStatus = TEMP_SENSOR_ERR;
    thermo.clearFault();
    return;
  }

  float t = thermo.temperature(RNOMINAL, RREF);

  if (t < -100.0f || t > 300.0f) {
    tempStatus = TEMP_SENSOR_ERR;
    return;
  }

  currentTemp = t;
  tempStatus = TEMP_OK;
#else
  tempStatus = TEMP_NO_MAX;
  currentTemp = 0.0f;
#endif
}

// ===================== TEXTAUSGABE SENSORSTATUS =====================
void makeTempText(char* out, size_t outSz) {
  if (tempStatus == TEMP_NO_MAX) {
    strncpy(out, "NO-MAX", outSz);
    out[outSz - 1] = '\0';
  } else if (tempStatus == TEMP_SENSOR_ERR) {
    strncpy(out, "PT100ERR", outSz);
    out[outSz - 1] = '\0';
  } else {
    char tempBuf[8];
    formatFloat(currentTemp, 4, 1, tempBuf, sizeof(tempBuf));
    snprintf(out, outSz, "%s%cC", tempBuf, 223);
  }
}
void makePressureText(char* out, size_t outSz) {
  if (pressureStatus == PRESS_NO_SENSOR) {
    strncpy(out, "NO-PRES", outSz);
  } 
  else if (pressureStatus == PRESS_SENSOR_ERR) {
    strncpy(out, "PRES-ERR", outSz);
  } 
  else {
    char tmp[10];
dtostrf(currentBar, 4, 1, tmp);

// führende Leerzeichen entfernen
while (tmp[0] == ' ') memmove(tmp, tmp+1, strlen(tmp));

snprintf(out, outSz, "%sbar", tmp);
  }

  out[outSz - 1] = '\0';
}

// ===================== WARN / ALARM =====================
void updatePressureStates() {
  ScrewProfile &sp = screws[currentScrew];

  // Wenn kein gültiger Druckwert da ist, keine Warnung/kein Alarm
  if (pressureStatus != PRESS_OK) {
    pressureWarnActive = false;
    pressureAlarmActive = false;
    return;
  }

  if (!pressureAlarmActive) {
    if (currentBar >= sp.alarmPressureBar) {
      pressureAlarmActive = true;
    }
  } else {
    if (currentBar < (sp.alarmPressureBar - PRESSURE_HYSTERESIS)) {
      pressureAlarmActive = false;
    }
  }

  if (!pressureWarnActive) {
    if (currentBar >= sp.warnPressureBar) {
      pressureWarnActive = true;
    }
  } else {
    if (currentBar < (sp.warnPressureBar - PRESSURE_HYSTERESIS)) {
      pressureWarnActive = false;
    }
  }

  if (pressureAlarmActive) {
    pressureWarnActive = false;
  }
}

void updateBuzzer() {
  static bool buzzPhase = false;
  static unsigned long lastBuzz = 0;

  if (pressureAlarmActive) {
    if (millis() - lastBuzz > 300) {
      lastBuzz = millis();
      buzzPhase = !buzzPhase;
      if (buzzPhase) {
        tone(PIN_BUZZER, 2200, 180);
      }
    }
  } else {
    noTone(PIN_BUZZER);
  }
}

// ===================== ANZEIGE =====================
void drawCompact() {
    lcd.backlight();
  int rpmInt = (int)(currentRPM + 0.5f);
  float lpm = currentRPM * screws[currentScrew].litersPerRev;

  char line1[17];
  char line2[17];

  // ===== Zeile 1 =====
  char rpmText[8];
  intToRpmText(rpmInt, rpmText, sizeof(rpmText));

  char pressurePart[16];
  bool blinkState = (millis() / 400) % 2;

  if (pressureWarnActive && blinkState && pressureStatus == PRESS_OK) {
    strcpy(pressurePart, "       ");
 } else {
  makePressureText(pressurePart, sizeof(pressurePart));
}

  // rpm links, Druck rechts
  snprintf(line1, sizeof(line1), "%-6s%9s", rpmText, pressurePart);

  // ===== Zeile 2 =====
  char lpmBuf[8];
  if (lpm < 9.995f) formatFloat(lpm, 4, 2, lpmBuf, sizeof(lpmBuf));
  else              formatFloat(lpm, 4, 1, lpmBuf, sizeof(lpmBuf));

  char tempText[10];
  makeTempText(tempText, sizeof(tempText));

  snprintf(line2, sizeof(line2), "%sl/min %s", lpmBuf, tempText);

  printPaddedLine(0, line1);
  printPaddedLine(1, line2);
}

void drawMenu() {
  lcd.noBlink();
  lcd.clear();
  printPaddedLine(0, "Menue:");
  printPaddedLine(1, menuItems[menuIndex]);
}

void drawSelectScrew() {
  lcd.noBlink();
  lcd.clear();
  printPaddedLine(0, "Schnecke wahl");

  bool blinkOn = (millis() / 400) % 2;
  char buf[17];
  if (blinkOn) snprintf(buf, sizeof(buf), "> %s <", screws[pendingScrew].name);
  else         snprintf(buf, sizeof(buf), "  %s  ", screws[pendingScrew].name);

  printPaddedLine(1, buf);
}

void drawEditFactor() {
  lcd.noBlink();
  lcd.clear();
  ScrewProfile &sp = screws[currentScrew];

  char line1[17];
  snprintf(line1, sizeof(line1), "%s Faktor:", sp.name);
  printPaddedLine(0, line1);

  char valBuf[10];
  formatFloat(sp.litersPerRev, 6, 3, valBuf, sizeof(valBuf));

  char line2[17];
  snprintf(line2, sizeof(line2), "%s L/U", valBuf);
  printPaddedLine(1, line2);
}

void drawEditName() {
  ScrewProfile &sp = screws[currentScrew];

  lcd.clear();
  printPaddedLine(0, "Name edit:");
  lcd.setCursor(0, 1);
  lcd.print(sp.name);
  for (int i = strlen(sp.name); i < 16; i++) lcd.print(' ');
  lcd.setCursor(nameEditPos, 1);
  lcd.blink();
}

void drawEditMagnets() {
  lcd.noBlink();
  lcd.clear();
  printPaddedLine(0, "Magnete/U:");

  char buf[17];
  snprintf(buf, sizeof(buf), "%d", (int)magnetsPerRev);
  printPaddedLine(1, buf);
}

void drawEditWarnPressure() {
  lcd.noBlink();
  lcd.clear();
  ScrewProfile &sp = screws[currentScrew];

  char line1[17];
  snprintf(line1, sizeof(line1), "%s Warnung:", sp.name);
  printPaddedLine(0, line1);

  char pBuf[10];
  formatFloat(sp.warnPressureBar, 5, 1, pBuf, sizeof(pBuf));

  char line2[17];
  snprintf(line2, sizeof(line2), "%s bar", pBuf);
  printPaddedLine(1, line2);
}

void drawEditAlarmPressure() {
  lcd.noBlink();
  lcd.clear();
  ScrewProfile &sp = screws[currentScrew];

  char line1[17];
  snprintf(line1, sizeof(line1), "%s Alarm:", sp.name);
  printPaddedLine(0, line1);

  char pBuf[10];
  formatFloat(sp.alarmPressureBar, 5, 1, pBuf, sizeof(pBuf));

  char line2[17];
  snprintf(line2, sizeof(line2), "%s bar", pBuf);
  printPaddedLine(1, line2);
}

// ===================== SCHNECKENVERWALTUNG =====================
void addScrew() {
  if (currentScrewCount < NUM_SCREWS) {
    strcpy(screws[currentScrewCount].name, "NEW");
    screws[currentScrewCount].litersPerRev = 0.000f;
    screws[currentScrewCount].warnPressureBar = 10.0f;
    screws[currentScrewCount].alarmPressureBar = 12.0f;
    pendingScrew = currentScrewCount;
    currentScrewCount++;
  } else {
    lcd.clear();
    printPaddedLine(0, "Max 5 Schnecken");
    printPaddedLine(1, "belegt");
    delay(1200);
  }
}

void deleteScrew(byte index) {
  if (index == 0) {
    lcd.clear();
    printPaddedLine(0, "D4-3 fix!");
    printPaddedLine(1, "nicht loeschbar");
    delay(1200);
    return;
  }

  if (index < currentScrewCount && currentScrewCount > 1) {
    for (byte i = index; i < currentScrewCount - 1; i++) {
      screws[i] = screws[i + 1];
    }
    currentScrewCount--;
    if (currentScrew >= currentScrewCount) currentScrew = currentScrewCount - 1;
    if (pendingScrew >= currentScrewCount) pendingScrew = currentScrewCount - 1;
  }
}

// ===================== SETUP =====================
void setup() {
  pinMode(PIN_ENCODER_CLK, INPUT_PULLUP);
  pinMode(PIN_ENCODER_DT,  INPUT_PULLUP);
  pinMode(PIN_ENCODER_SW,  INPUT_PULLUP);
  pinMode(PIN_OPTO,        INPUT);
  pinMode(PIN_BUZZER,      OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  lastCLKState  = digitalRead(PIN_ENCODER_CLK);
  lastOptoState = digitalRead(PIN_OPTO);

  lcd.init();
  lcd.backlight();

#if USE_PT100
  thermo.begin(MAX31865_3WIRE);
  tempStatus = TEMP_SENSOR_ERR;
#else
  tempStatus = TEMP_NO_MAX;
#endif

  lcd.clear();
  printPaddedLine(0, "Niromat 3DCP-LT1");
  printPaddedLine(1, "by G. Kordowich");
  delay(3500);

  lcd.clear();
  printPaddedLine(0, "THWS RoeRing");
  printPaddedLine(1, "Baustofflabor");
  delay(3500);

  lcd.clear();
  printPaddedLine(0, "Foerder-Anzeige");
  char s2[17];
  snprintf(s2, sizeof(s2), "Start mit %s", screws[currentScrew].name);
  printPaddedLine(1, s2);
  delay(2500);

  lcd.clear();
  lastPulseMicros = micros();
  pendingScrew = currentScrew;
  drawCompact();
}

// ===================== LOOP =====================
void drawAlarmScreen() {
  lcd.clear();

  // Zeile 1: Alarmtext
  printPaddedLine(0, "!!! UEBERDRUCK !!!");

  // Zeile 2: Druck + Temperatur
  char line2[17];

  char pressureText[10];
  char tempText[10];

  makePressureText(pressureText, sizeof(pressureText));
  makeTempText(tempText, sizeof(tempText));

  snprintf(line2, sizeof(line2), "%s %s", pressureText, tempText);
  printPaddedLine(1, line2);
  static unsigned long lastBlink = 0;
static bool lightOn = true;

if (millis() - lastBlink > (lightOn ? 700 : 300)) {
  lastBlink = millis();
  lightOn = !lightOn;
}

if (lightOn) lcd.backlight();
else lcd.noBacklight();

}
void loop() {
  measureRPM();
  measurePressure();
  measureTemp();
  updatePressureStates();
  updateBuzzer();
  updateEncoder();

  if (pressureAlarmActive && uiMode == UI_VIEW) {
    static unsigned long lastAlarmDisp = 0;
    if (millis() - lastAlarmDisp > 250) {
      drawAlarmScreen();
      lastAlarmDisp = millis();
    }

    if (buttonPressed()) {
      uiMode = UI_MENU;
      encoderPos = 0;
      menuIndex = 0;
      drawMenu();
    }
    return;
  }

  if (uiMode == UI_VIEW) {
    if (buttonPressed()) {
      uiMode = UI_MENU;
      encoderPos = 0;
      menuIndex = 0;
      drawMenu();
    }

    static unsigned long lastDisp = 0;
    if (millis() - lastDisp > 500) {
      drawCompact();
      lastDisp = millis();
    }

  } else if (uiMode == UI_MENU) {

    if (encoderPos >= 1) {
      if (menuIndex < NUM_MENU_ITEMS - 1) menuIndex++;
      encoderPos = 0;
      drawMenu();
    } else if (encoderPos <= -1) {
      if (menuIndex > 0) menuIndex--;
      encoderPos = 0;
      drawMenu();
    }

    if (buttonPressed()) {
      switch (menuIndex) {
        case 0:
          uiMode = UI_SELECT_SCREW;
          encoderPos = 0;
          pendingScrew = currentScrew;
          drawSelectScrew();
          break;

        case 1:
          uiMode = UI_EDIT_FACTOR;
          encoderPos = 0;
          drawEditFactor();
          break;

        case 2:
          uiMode = UI_EDIT_NAME;
          encoderPos = 0;
          nameEditPos = 0;
          drawEditName();
          break;

        case 3:
          uiMode = UI_EDIT_MAGNETS;
          encoderPos = 0;
          drawEditMagnets();
          break;

        case 4:
          uiMode = UI_EDIT_WARN_PRESSURE;
          encoderPos = 0;
          drawEditWarnPressure();
          break;

        case 5:
          uiMode = UI_EDIT_ALARM_PRESSURE;
          encoderPos = 0;
          drawEditAlarmPressure();
          break;

        case 6:
          addScrew();
          uiMode = UI_SELECT_SCREW;
          encoderPos = 0;
          drawSelectScrew();
          break;

        case 7:
          deleteScrew(currentScrew);
          uiMode = UI_VIEW;
          encoderPos = 0;
          drawCompact();
          break;

        case 8:
        default:
          uiMode = UI_VIEW;
          encoderPos = 0;
          drawCompact();
          break;
      }
    }

  } else if (uiMode == UI_SELECT_SCREW) {

    if (encoderPos >= 1) {
      if (pendingScrew < currentScrewCount - 1) pendingScrew++;
      encoderPos = 0;
      drawSelectScrew();
    } else if (encoderPos <= -1) {
      if (pendingScrew > 0) pendingScrew--;
      encoderPos = 0;
      drawSelectScrew();
    }

    if (buttonPressed()) {
      currentScrew = pendingScrew;
      uiMode = UI_VIEW;
      encoderPos = 0;
      drawCompact();
    }

    static unsigned long lastBlinkRedraw = 0;
    if (millis() - lastBlinkRedraw > 200) {
      drawSelectScrew();
      lastBlinkRedraw = millis();
    }

  } else if (uiMode == UI_EDIT_FACTOR) {
    ScrewProfile &sp = screws[currentScrew];

    if (encoderPos >= 1) {
      sp.litersPerRev += 0.001f;
      encoderPos = 0;
      drawEditFactor();
    } else if (encoderPos <= -1) {
      if (sp.litersPerRev > 0.001f) sp.litersPerRev -= 0.001f;
      encoderPos = 0;
      drawEditFactor();
    }

    if (buttonPressed()) {
      uiMode = UI_MENU;
      encoderPos = 0;
      drawMenu();
    }

  } else if (uiMode == UI_EDIT_NAME) {
    ScrewProfile &sp = screws[currentScrew];

    if (encoderPos >= 1) {
      sp.name[nameEditPos] = nextChar(sp.name[nameEditPos], +1);
      encoderPos = 0;
      drawEditName();
    } else if (encoderPos <= -1) {
      sp.name[nameEditPos] = nextChar(sp.name[nameEditPos], -1);
      encoderPos = 0;
      drawEditName();
    }

    sp.name[8] = '\0';

    if (buttonPressed()) {
      nameEditPos++;
      if (nameEditPos >= 8) {
        lcd.noBlink();
        uiMode = UI_MENU;
        encoderPos = 0;
        drawMenu();
      } else {
        drawEditName();
      }
    }

  } else if (uiMode == UI_EDIT_MAGNETS) {

    if (encoderPos >= 1) {
      if (magnetsPerRev < 8) magnetsPerRev++;
      encoderPos = 0;
      drawEditMagnets();
    } else if (encoderPos <= -1) {
      if (magnetsPerRev > 1) magnetsPerRev--;
      encoderPos = 0;
      drawEditMagnets();
    }

    if (buttonPressed()) {
      uiMode = UI_MENU;
      encoderPos = 0;
      drawMenu();
    }

  } else if (uiMode == UI_EDIT_WARN_PRESSURE) {
    ScrewProfile &sp = screws[currentScrew];

    if (encoderPos >= 1) {
      sp.warnPressureBar += 0.1f;
      if (sp.warnPressureBar > sp.alarmPressureBar) sp.warnPressureBar = sp.alarmPressureBar;
      encoderPos = 0;
      drawEditWarnPressure();
    } else if (encoderPos <= -1) {
      if (sp.warnPressureBar > 0.1f) sp.warnPressureBar -= 0.1f;
      encoderPos = 0;
      drawEditWarnPressure();
    }

    if (buttonPressed()) {
      uiMode = UI_MENU;
      encoderPos = 0;
      drawMenu();
    }

  } else if (uiMode == UI_EDIT_ALARM_PRESSURE) {
    ScrewProfile &sp = screws[currentScrew];

    if (encoderPos >= 1) {
      sp.alarmPressureBar += 0.1f;
      encoderPos = 0;
      drawEditAlarmPressure();
    } else if (encoderPos <= -1) {
      if (sp.alarmPressureBar > (sp.warnPressureBar + 0.1f)) sp.alarmPressureBar -= 0.1f;
      encoderPos = 0;
      drawEditAlarmPressure();
    }

    if (buttonPressed()) {
      uiMode = UI_MENU;
      encoderPos = 0;
      drawMenu();
    }
  }
}

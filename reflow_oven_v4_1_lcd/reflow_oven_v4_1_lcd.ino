/*
 * ============================================================
 *  VIDHYUT RAKSHAK — REFLOW OVEN CONTROLLER  v4.1-LCD
 *  ESP32 + MAX31855 Thermocouple + 3 Buttons + 16×2 I2C LCD
 *
 *  LCD DISPLAY LAYOUT
 *  ─────────────────
 *  MODE SELECT screen:
 *    Line 1: [M1] SAC305
 *    Line 2: >START  ABT  <<MODE
 *
 *  IDLE screen (after mode confirmed):
 *    Line 1: CT=xxx.x C  M1
 *    Line 2: >START  ABT  CHG
 *
 *  RUNNING screen:
 *    Line 1: CT=xxx.x PREHEAT
 *    Line 2: >ABORT    t+xxxs
 *
 *  COOLING screen:
 *    Line 1: CT=xxx.x COOLING
 *    Line 2: Fan ON  Wait...
 *
 *  DONE screen:
 *    Line 1: CT=xxx.x  DONE!
 *    Line 2: >START again  CHG
 *
 *  FAULT screen:
 *    Line 1: !! FAULT !!
 *    Line 2: >ABT  Check TC
 *
 *  DOUBLE-CLICK on MODE button cycles through modes M1→M2→M3→M4→M1
 * ============================================================
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Adafruit_MAX31855.h"
#include "esp_task_wdt.h"

#define PLOTTER_MODE 0
#define DLOG(x) do { if (!PLOTTER_MODE) { x; } } while(0)

// ── LCD (I2C) ─────────────────────────────────────────────────
// Default I2C address is 0x27; change to 0x3F if needed
#define LCD_ADDR  0x27
#define LCD_COLS  16
#define LCD_ROWS   2
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// ── THERMOCOUPLE (VSPI) ───────────────────────────────────────
#define MAXDO    19
#define MAXCS     5
#define MAXCLK   18
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

// ── OTHER HARDWARE ────────────────────────────────────────────
#define SSR_PIN   4
#define FAN_PIN  25
#define BTN_START  26
#define BTN_ABORT  27
#define BTN_MODE   14

#define FAN_LEDC_CH    0
#define FAN_LEDC_FREQ  25000
#define FAN_LEDC_BITS  8

// ═════════════════════════════════════════════════════════════
//  MULTI-MODE PROFILE DATA
// ═════════════════════════════════════════════════════════════
const int NUM_POINTS = 6;

const float modeTemps[3][NUM_POINTS] = {
  { 25.0, 150.0, 165.0, 185.0, 240.0, 25.0 },  // M1 SAC305
  { 25.0, 150.0, 165.0, 183.0, 210.0, 25.0 },  // M2 Sn63Pb37
  { 25.0, 100.0, 130.0, 138.0, 170.0, 25.0 },  // M3 LowTemp
};
const int modeSoaks[3][NUM_POINTS] = {
  { 0, 60, 30, 30, 25, 0 },
  { 0, 60, 30, 30, 20, 0 },
  { 0, 45, 30, 45, 20, 0 },
};
const int modeTimeouts[3][NUM_POINTS] = {
  { 60, 480, 300, 300, 600, 900 },
  { 60, 480, 300, 300, 500, 900 },
  { 60, 360, 300, 300, 480, 900 },
};

const char* modeNames[3]    = { "SAC305",   "Sn63Pb37", "LowTemp" };
const char* modeShort[3]    = { "SAC305",   "Sn63Pb",   "LowTmp"  };
const float modeLiquidus[3] = { 183.0, 183.0, 138.0 };
const float modeOverTemp[3] = { 280.0, 250.0, 200.0 };

const char* waypointLabel[NUM_POINTS] = {
  "START", "PREHEAT", "SOAK1", "SOAK2", "PEAK", "COOLING"
};
// Short 7-char versions for LCD line 1 (leaves room for temp)
const char* waypointShort[NUM_POINTS] = {
  "START  ", "PREHEAT", "SOAK-1 ", "SOAK-2 ", "PEAK   ", "COOLING"
};

// Custom mode storage
float  customLiquidus = 183.0;
float  customPeakTemp = 240.0;
char   customModeName[8] = "Custom";

// Working profile arrays
float waypointTemp[NUM_POINTS];
int   waypointSoak[NUM_POINTS];
int   waypointTimeout[NUM_POINTS];
uint8_t selectedMode = 1;

// ── Safe accessors (mode 1–4, arrays 0-indexed 0–2) ──────────
const char* getModeName() {
  if (selectedMode >= 1 && selectedMode <= 3) return modeNames[selectedMode - 1];
  return customModeName;
}
const char* getModeShort() {
  if (selectedMode >= 1 && selectedMode <= 3) return modeShort[selectedMode - 1];
  return "Custom";
}
float getModeLiquidus() {
  if (selectedMode >= 1 && selectedMode <= 3) return modeLiquidus[selectedMode - 1];
  return customLiquidus;
}
float getModeOverTemp() {
  if (selectedMode >= 1 && selectedMode <= 3) return modeOverTemp[selectedMode - 1];
  return customPeakTemp + 40.0;
}

// ── SSR ───────────────────────────────────────────────────────
const int WINDOW_MS = 2000;

// ── GLITCH FILTER ─────────────────────────────────────────────
const float GLITCH_ZERO_MAX   = 5.0;
const float GLITCH_RATE_HEAT  = 15.0;
const float GLITCH_RATE_COOL  = 35.0;
const int   GLITCH_MAX_CONSEC = 3;

// ── STATE ─────────────────────────────────────────────────────
enum CycleState { IDLE, RAMPING, SOAKING, COOLING_DOWN, DONE, FAULT };
CycleState cycleState = IDLE;

bool   selectingMode = true;
float  overTemp      = 280.0;

int    waypointIndex  = 0;
int    soakCounter    = 0;
int    glitchCounter  = 0;
int    totalElapsed   = 0;

float  lastValidTemp  = 25.0;
double lastInput      = 25.0;
double integral       = 0.0;
double Output         = 0.0;
double Setpoint       = 25.0;

bool   isRunning   = false;
bool   fanForced   = false;
float  curKp = 0, curKi = 0, curKd = 0;

unsigned long windowStartTime   = 0;
unsigned long lastTickTime      = 0;
unsigned long lastPrintTime     = 0;
unsigned long waypointStartTime = 0;
unsigned long cycleStartTime    = 0;

// ── LCD UPDATE ────────────────────────────────────────────────
unsigned long lastLcdUpdate  = 0;
bool          lcdNeedsRedraw = true;
// Track last drawn strings so we only update changed chars
char lcdLine0[17] = "";
char lcdLine1[17] = "";

// ── BUTTONS ───────────────────────────────────────────────────
struct Btn {
  uint8_t pin;
  bool    lastState;
  bool    pressed;
  unsigned long lastChange;
  // Double-click tracking
  unsigned long lastPressTime;
  int           clickCount;
};
Btn btnStart = {BTN_START, HIGH, false, 0, 0, 0};
Btn btnAbort = {BTN_ABORT, HIGH, false, 0, 0, 0};
Btn btnMode  = {BTN_MODE,  HIGH, false, 0, 0, 0};

#define DEBOUNCE_MS      40
#define DBLCLICK_MS     400   // max gap between two clicks for double-click

bool idleReady    = false;
bool modeDblClick = false;    // set when MODE double-clicked

// ── PID ───────────────────────────────────────────────────────
struct PIDGains { double Kp, Ki, Kd; };

// ── AI STATE ──────────────────────────────────────────────────
double        aiCorrection    = 0.0;
double        aiCorrectionInt = 0.0;
unsigned long aiCorrLastRx    = 0;
const unsigned long AI_RX_TIMEOUT   = 5000;
const int           AI_MAX_SAFE_CORR = 600;
const int           AI_MIN_SAFE_CORR = -400;
const double        AI_RAMP_STEP    = 5.0;
int                 aiErrorCount    = 0;
bool                aiActive        = true;

// ── FORWARD DECLARATIONS ──────────────────────────────────────
void setSSR(bool on);
void setFan(int speed);
void advanceWaypoint(const char* reason);
float readTempFiltered();
void updateButtons();
void handleButtons();
void updateLcd();
void lcdPrint(uint8_t row, const char* text);
void abortCycle();
void startCycle(float bootTemp);
void loadProfile(int modeIdx);
void logSerial(float temp);
PIDGains getZoneGains(float temp, float setpoint);

// ─────────────────────────────────────────────────────────────
//  LCD HELPER — only writes when content changes to avoid flicker
// ─────────────────────────────────────────────────────────────
void lcdPrint(uint8_t row, const char* text) {
  // Pad to 16 chars
  char padded[17];
  snprintf(padded, 17, "%-16s", text);
  char* cached = (row == 0) ? lcdLine0 : lcdLine1;
  if (strncmp(padded, cached, 16) == 0) return;  // no change
  strncpy(cached, padded, 16);
  cached[16] = '\0';
  lcd.setCursor(0, row);
  lcd.print(padded);
}

// ─────────────────────────────────────────────────────────────
//  LOAD PROFILE
// ─────────────────────────────────────────────────────────────
void loadProfile(int m) {
  if (m >= 0 && m < 3) {
    for (int i = 0; i < NUM_POINTS; i++) {
      waypointTemp[i]    = modeTemps[m][i];
      waypointSoak[i]    = modeSoaks[m][i];
      waypointTimeout[i] = modeTimeouts[m][i];
    }
    overTemp = modeOverTemp[m];
  } else {
    // Custom mode
    waypointTemp[0] = 25.0;
    waypointTemp[1] = 150.0;
    waypointTemp[2] = (customLiquidus > 150.0) ? (customLiquidus - 33.0) : 120.0;
    waypointTemp[3] = customLiquidus;
    waypointTemp[4] = customPeakTemp;
    waypointTemp[5] = 25.0;
    for (int i = 0; i < NUM_POINTS; i++) {
      waypointSoak[i]    = (int[]){0, 60, 30, 30, 25, 0}[i];
      waypointTimeout[i] = (int[]){60, 480, 300, 300, 600, 900}[i];
    }
    overTemp = customPeakTemp + 40.0;
  }
}

// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(300);

  esp_task_wdt_init(12, true);
  esp_task_wdt_add(NULL);

  // ── Hardware init ─────────────────────────────────────────
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);
  ledcSetup(FAN_LEDC_CH, FAN_LEDC_FREQ, FAN_LEDC_BITS);
  ledcAttachPin(FAN_PIN, FAN_LEDC_CH);
  ledcWrite(FAN_LEDC_CH, 0);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_ABORT, INPUT_PULLUP);
  pinMode(BTN_MODE,  INPUT_PULLUP);

  // ── LCD init ──────────────────────────────────────────────
  Wire.begin();           // SDA=21, SCL=22 default on ESP32
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Splash screen
  lcd.setCursor(0, 0); lcd.print("VIDHYUT RAKSHAK ");
  lcd.setCursor(0, 1); lcd.print("Reflow Ctrl v4.1");
  delay(1500);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Initialising... ");

  delay(600);

  float bootTemp = thermocouple.readCelsius();
  if (isnan(bootTemp) || bootTemp < 5.0) bootTemp = 25.0;
  lastValidTemp = bootTemp;
  lastInput     = bootTemp;

  // Show boot temperature
  char buf[17];
  snprintf(buf, 17, "Boot:%-6.1f C   ", bootTemp);
  lcd.setCursor(0, 0); lcd.print(buf);

  if (bootTemp > 60.0) {
    lcd.setCursor(0, 1); lcd.print("Too hot!Cooling.");
    setFan(255);
    fanForced     = true;
    selectingMode = false;
    idleReady     = false;
  } else {
    lcd.setCursor(0, 1); lcd.print("                ");
  }
  delay(1000);

  loadProfile(0);   // default M1
  lcdNeedsRedraw = true;

  Serial.println(F("=== VIDHYUT RAKSHAK v4.1-LCD ready ==="));
  Serial.println(F("Commands: stop|status|mode1-4|custom:Liq:Peak|ai:+NNN|ai_status"));
}

// ═════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════
void loop() {
  esp_task_wdt_reset();

  updateButtons();
  handleButtons();

  // ── SERIAL COMMANDS ──────────────────────────────────────
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    String cmdLower = cmd;
    cmdLower.toLowerCase();

    if (cmdLower == "stop") {
      abortCycle();

    } else if (cmdLower == "status") {
      Serial.print(F("[STATUS] phase=")); Serial.print(cycleState);
      Serial.print(F(" wp="));           Serial.print(waypointIndex);
      Serial.print(F(" temp="));         Serial.print(lastValidTemp, 1);
      Serial.print(F(" target="));       Serial.print(Setpoint, 1);
      Serial.print(F(" out="));          Serial.print((int)Output);
      Serial.print(F(" mode=M"));        Serial.print(selectedMode);
      Serial.print(F(" ai="));           Serial.println(aiActive ? "on" : "off");

    } else if (cmdLower == "mode1") { selectedMode=1; loadProfile(0); selectingMode=true; lcdNeedsRedraw=true; }
    else if (cmdLower == "mode2")   { selectedMode=2; loadProfile(1); selectingMode=true; lcdNeedsRedraw=true; }
    else if (cmdLower == "mode3")   { selectedMode=3; loadProfile(2); selectingMode=true; lcdNeedsRedraw=true; }
    else if (cmdLower == "mode4")   { selectedMode=4; loadProfile(3); selectingMode=true; lcdNeedsRedraw=true; }

    else if (cmdLower.startsWith("custom:")) {
      String rest = cmd.substring(7);
      int sep = rest.indexOf(':');
      if (sep > 0) {
        customLiquidus = rest.substring(0, sep).toFloat();
        customPeakTemp = rest.substring(sep + 1).toFloat();
        if (customPeakTemp > customLiquidus && customLiquidus > 50.0 && customPeakTemp < 350.0) {
          loadProfile(3);
          selectedMode = 4;
          Serial.print(F("[CUSTOM] Liq=")); Serial.print(customLiquidus, 0);
          Serial.print(F(" Peak=")); Serial.println(customPeakTemp, 0);
        } else {
          Serial.println(F("[CUSTOM] Invalid: Peak must be > Liquidus"));
        }
      }

    } else if (cmdLower.startsWith("ai:")) {
      String valStr = cmd.substring(3);
      int parsed = atoi(valStr.c_str());
      if (parsed > AI_MAX_SAFE_CORR || parsed < AI_MIN_SAFE_CORR) {
        aiErrorCount++;
        if (aiErrorCount > 10) { aiActive = false; Serial.println(F("[AI_FAULT] Disabled")); }
      } else {
        aiErrorCount = 0;
        aiCorrection = (double)parsed;
        aiCorrLastRx = millis();
      }

    } else if (cmdLower == "ai_status") {
      Serial.print(F("[AI_STATUS] active=")); Serial.print(aiActive?"yes":"no");
      Serial.print(F(" corr=")); Serial.println((int)aiCorrectionInt);
    }
  }

  unsigned long now = millis();

  // ── HOT-BOOT COOLDOWN ─────────────────────────────────────
  if (!isRunning && fanForced && !selectingMode) {
    float t = thermocouple.readCelsius();
    if (!isnan(t) && t < 55.0) {
      setFan(0); fanForced = false; selectingMode = true; lcdNeedsRedraw = true;
    }
    if (now - lastLcdUpdate >= 500) { updateLcd(); lastLcdUpdate = now; }
    delay(10); return;
  }

  if (selectingMode) {
    if (now - lastLcdUpdate >= 500) { updateLcd(); lastLcdUpdate = now; }
    delay(10); return;
  }

  // ── POST-CYCLE FAN RUNDOWN ────────────────────────────────
  if (!isRunning && fanForced) {
    float t = thermocouple.readCelsius();
    if (!isnan(t) && t < 55.0) {
      setFan(0); fanForced = false; idleReady = true; lcdNeedsRedraw = true;
    }
    if (now - lastLcdUpdate >= 500) { updateLcd(); lastLcdUpdate = now; }
    delay(10); return;
  }

  if (!isRunning) {
    if (now - lastLcdUpdate >= 1000) {
      float t = thermocouple.readCelsius();
      if (!isnan(t) && t > 5.0) lastValidTemp = t;
      updateLcd(); lastLcdUpdate = now;
    }
    delay(10); return;
  }

  // ── SSR TIME PROPORTIONING ────────────────────────────────
  unsigned long winElapsed = now - windowStartTime;
  if (winElapsed >= WINDOW_MS) { windowStartTime = now; winElapsed = 0; }
  setSSR(Output > (double)winElapsed);

  // ── AI RAMP SMOOTHING ─────────────────────────────────────
  if (aiActive && isRunning && cycleState != COOLING_DOWN) {
    double diff = aiCorrection - aiCorrectionInt;
    if (abs(diff) <= AI_RAMP_STEP) aiCorrectionInt = aiCorrection;
    else aiCorrectionInt += (diff > 0 ? AI_RAMP_STEP : -AI_RAMP_STEP);
  } else {
    if (abs(aiCorrectionInt) < AI_RAMP_STEP) aiCorrectionInt = 0;
    else aiCorrectionInt -= (aiCorrectionInt > 0 ? AI_RAMP_STEP : -AI_RAMP_STEP);
  }

  // ── 1-SECOND TICK ─────────────────────────────────────────
  if (now - lastTickTime < 1000) {
    if (now - lastLcdUpdate >= 200) { updateLcd(); lastLcdUpdate = now; }
    delay(5); return;
  }
  lastTickTime = now;
  totalElapsed = (int)((now - cycleStartTime) / 1000);

  // ── AI TIMEOUT ────────────────────────────────────────────
  if (aiActive && (now - aiCorrLastRx) > AI_RX_TIMEOUT && aiCorrection != 0.0) {
    aiCorrection = 0.0;
  }

  // ── A. READ TEMPERATURE ───────────────────────────────────
  float temp = readTempFiltered();
  if (temp < 0) {
    setSSR(false); setFan(255);
    isRunning = false; cycleState = FAULT;
    Serial.println(F("[FAULT] Thermocouple fault.")); lcdNeedsRedraw = true; return;
  }

  // ── B. OVER-TEMP ──────────────────────────────────────────
  if (temp > overTemp) {
    setSSR(false); setFan(255);
    isRunning = false; cycleState = FAULT;
    Serial.print(F("[SAFETY] Over-temp ")); Serial.println(temp);
    lcdNeedsRedraw = true; return;
  }

  // ── C. WAYPOINT TIMEOUT ───────────────────────────────────
  int wpElapsed = (int)((now - waypointStartTime) / 1000);
  if (cycleState == RAMPING && wpElapsed > waypointTimeout[waypointIndex]) {
    advanceWaypoint("timeout"); return;
  }

  // ── D. STATE MACHINE ──────────────────────────────────────
  if (cycleState == RAMPING) {
    Setpoint = waypointTemp[waypointIndex];
    if (temp >= Setpoint - 5.0) {
      cycleState = SOAKING; soakCounter = 0; integral = 0.0;
      lcdNeedsRedraw = true;
    }

  } else if (cycleState == SOAKING) {
    soakCounter++;
    if (soakCounter >= waypointSoak[waypointIndex]) {
      advanceWaypoint("soak_complete"); return;
    }

  } else if (cycleState == COOLING_DOWN) {
    setSSR(false); Output = 0;
    if (temp <= 55.0) {
      setFan(0); fanForced = false; isRunning = false;
      cycleState = DONE; idleReady = true; lcdNeedsRedraw = true;
      Serial.println(F("[DONE] Cycle complete."));
    }
    logSerial(temp); return;
  }

  // ── E. ZONE PID ───────────────────────────────────────────
  PIDGains g = getZoneGains(temp, Setpoint);
  curKp = g.Kp; curKi = g.Ki; curKd = g.Kd;

  double error = Setpoint - temp;
  integral += error;
  double windupLimit = (abs(error) < 10.0) ? 80.0 : 200.0;
  integral = constrain(integral, -windupLimit, windupLimit);
  double derivative = -(temp - lastInput);
  lastInput = temp;
  double pidOutput = (g.Kp * error) + (g.Ki * integral) + (g.Kd * derivative);

  double appliedCorr = aiCorrectionInt;
  if (temp > overTemp * 0.90) appliedCorr = 0.0;
  if (cycleState == COOLING_DOWN || cycleState == DONE) appliedCorr = 0.0;
  Output = constrain(pidOutput + appliedCorr, 0.0, (double)WINDOW_MS);

  // ── F. FAN ────────────────────────────────────────────────
  int fanSpeed = 0;
  if (cycleState == COOLING_DOWN)  fanSpeed = 255;
  else if (temp > Setpoint + 5.0)  fanSpeed = 200;
  setFan(fanSpeed);

  // ── G. SERIAL ─────────────────────────────────────────────
  if (now - lastPrintTime >= 2000) {
    lastPrintTime = now;
    logSerial(temp);
  }

  if (now - lastLcdUpdate >= 500) { updateLcd(); lastLcdUpdate = now; }
}

// ═════════════════════════════════════════════════════════════
//  LCD UPDATE — all screen states
// ═════════════════════════════════════════════════════════════
void updateLcd() {
  char l0[17], l1[17];

  // ── HOT-BOOT COOL-DOWN ────────────────────────────────────
  if (fanForced && !selectingMode && !isRunning) {
    snprintf(l0, 17, "CT=%-5.1f COOLING", lastValidTemp);
    snprintf(l1, 17, "Fan ON  Wait... ");
    lcdPrint(0, l0); lcdPrint(1, l1);
    return;
  }

  // ── MODE SELECT ───────────────────────────────────────────
  if (selectingMode) {
    // Line 0: show mode number + name
    if (selectedMode <= 3) {
      snprintf(l0, 17, "[M%d] %-11s", selectedMode, modeNames[selectedMode - 1]);
    } else {
      snprintf(l0, 17, "[M4] Custom L%3.0fP%3.0f",
               customLiquidus, customPeakTemp);
    }
    // Line 1: button guide
    // MODE btn: double-click = cycle mode, START = confirm, ABORT = back
    snprintf(l1, 17, "STR=OK ABT <<MOD");
    lcdPrint(0, l0); lcdPrint(1, l1);
    return;
  }

  // ── FAULT ─────────────────────────────────────────────────
  if (cycleState == FAULT) {
    snprintf(l0, 17, "!! FAULT !!     ");
    snprintf(l1, 17, "ABT=Reset ChkTC ");
    lcdPrint(0, l0); lcdPrint(1, l1);
    return;
  }

  // ── COOLING DOWN (post-peak fan rundown) ──────────────────
  if (cycleState == COOLING_DOWN || (fanForced && !isRunning)) {
    snprintf(l0, 17, "CT=%-5.1f COOLING", lastValidTemp);
    snprintf(l1, 17, "ABT=Stop  t+%ds ", totalElapsed);
    lcdPrint(0, l0); lcdPrint(1, l1);
    return;
  }

  // ── DONE ──────────────────────────────────────────────────
  if (cycleState == DONE) {
    snprintf(l0, 17, "CT=%-5.1f  DONE! ", lastValidTemp);
    snprintf(l1, 17, "STR=Again CHG-M ");
    lcdPrint(0, l0); lcdPrint(1, l1);
    return;
  }

  // ── IDLE (mode confirmed, waiting for START) ──────────────
  if (!isRunning && idleReady) {
    snprintf(l0, 17, "CT=%-5.1f  M%d    ", lastValidTemp, selectedMode);
    snprintf(l1, 17, "STR=Go ABT CHG-M");
    lcdPrint(0, l0); lcdPrint(1, l1);
    return;
  }

  // ── RUNNING ───────────────────────────────────────────────
  if (isRunning) {
    // Line 0: CT=xxx.x <WAYPOINT>
    // waypointShort is 7 chars, temp field is "xxx.x" (5 chars)
    // "CT=xxx.x " = 9 chars + 7 waypoint = 16 total
    snprintf(l0, 17, "CT=%-5.1f %s",
             lastValidTemp,
             waypointShort[waypointIndex]);

    // Line 1: context-sensitive
    if (cycleState == SOAKING && waypointSoak[waypointIndex] > 0) {
      // Show soak countdown
      int soakLeft = waypointSoak[waypointIndex] - soakCounter;
      snprintf(l1, 17, "ABT Soak:%-3ds   ", soakLeft);
    } else if (cycleState == RAMPING) {
      // Show target temperature
      snprintf(l1, 17, "ABT Tgt:%-3.0fC %3ds",
               Setpoint,
               (int)((millis() - waypointStartTime) / 1000));
    } else {
      snprintf(l1, 17, "ABT       t+%4ds", totalElapsed);
    }
    lcdPrint(0, l0); lcdPrint(1, l1);
    return;
  }

  // ── FALLBACK IDLE ─────────────────────────────────────────
  snprintf(l0, 17, "CT=%-5.1f  IDLE  ", lastValidTemp);
  snprintf(l1, 17, "STR=Start MOD=Md");
  lcdPrint(0, l0); lcdPrint(1, l1);
}

// ═════════════════════════════════════════════════════════════
//  BUTTON HANDLING — edge-triggered debounce + double-click
// ═════════════════════════════════════════════════════════════
void updateButtons() {
  unsigned long now = millis();
  modeDblClick = false;

  auto scan = [&](Btn &b) {
    bool cur = digitalRead(b.pin);
    b.pressed = false;
    if (cur != b.lastState) {
      if ((now - b.lastChange) > DEBOUNCE_MS) {
        b.lastChange = now;
        b.lastState  = cur;
        if (cur == LOW) {   // falling edge = pressed
          b.pressed = true;
        }
      }
    }
  };
  scan(btnStart); scan(btnAbort);

  // Scan MODE with double-click detection
  {
    Btn &b = btnMode;
    bool cur = digitalRead(b.pin);
    b.pressed = false;
    if (cur != b.lastState) {
      if ((now - b.lastChange) > DEBOUNCE_MS) {
        b.lastChange = now;
        b.lastState  = cur;
        if (cur == LOW) {   // falling edge
          b.pressed = true;
          b.clickCount++;
          if (b.clickCount == 1) {
            b.lastPressTime = now;
          } else if (b.clickCount == 2) {
            if ((now - b.lastPressTime) <= DBLCLICK_MS) {
              modeDblClick = true;
            }
            b.clickCount = 0;
          }
        }
      }
    }
    // Reset click count if timeout expires
    if (b.clickCount == 1 && (now - b.lastPressTime) > DBLCLICK_MS) {
      b.clickCount = 0;
    }
  }
}

void handleButtons() {
  // ── MODE SELECT screen ────────────────────────────────────
  if (selectingMode) {
    // Double-click MODE = cycle through modes
    if (modeDblClick) {
      selectedMode = (selectedMode % 4) + 1;
      loadProfile(selectedMode - 1);
      lcdNeedsRedraw = true;
      Serial.print(F("[MODE] Switched to M")); Serial.println(selectedMode);
    }
    // START = confirm selected mode
    if (btnStart.pressed) {
      selectingMode = false; idleReady = true; lcdNeedsRedraw = true;
      Serial.print(F("[MODE] Confirmed M")); Serial.println(selectedMode);
    }
    // ABORT = just refresh (no action in select screen; can't go further back)
    return;
  }

  // ── IDLE screen ───────────────────────────────────────────
  if (!isRunning) {
    // START = begin cycle
    if (btnStart.pressed && idleReady) {
      float t = thermocouple.readCelsius();
      if (isnan(t)) t = 25.0;
      startCycle(t);
    }
    // ABORT = go back to mode select
    if (btnAbort.pressed) {
      selectingMode = true; lcdNeedsRedraw = true;
      Serial.println(F("[BTN] Back to mode select"));
    }
    // Double-click MODE = cycle mode while idle
    if (modeDblClick) {
      selectedMode = (selectedMode % 4) + 1;
      loadProfile(selectedMode - 1);
      lcdNeedsRedraw = true;
      Serial.print(F("[MODE] Changed to M")); Serial.println(selectedMode);
    }
    return;
  }

  // ── RUNNING screen ────────────────────────────────────────
  if (isRunning) {
    // ABORT = stop cycle
    if (btnAbort.pressed) {
      abortCycle();
    }
    // START does nothing while running (prevents accidental trigger)
    // MODE double-click does nothing while running
    return;
  }
}

// ═════════════════════════════════════════════════════════════
//  CYCLE CONTROL
// ═════════════════════════════════════════════════════════════
void startCycle(float bootTemp) {
  waypointIndex = 1;
  for (int i = 1; i < NUM_POINTS - 1; i++) {
    if (bootTemp >= waypointTemp[i] - 5.0) waypointIndex = i + 1;
    else break;
  }
  if (waypointIndex >= NUM_POINTS - 1) {
    waypointIndex = NUM_POINTS - 1;
    cycleState = COOLING_DOWN;
  } else {
    Setpoint = waypointTemp[waypointIndex];
    if (bootTemp >= Setpoint - 5.0) {
      cycleState = SOAKING; soakCounter = 0; integral = 0.0;
    } else {
      cycleState = RAMPING;
    }
  }

  isRunning = true; fanForced = false; idleReady = false;
  if (cycleState == RAMPING) integral = 0.0;
  lastInput         = bootTemp;
  cycleStartTime    = millis();
  waypointStartTime = millis();
  lastTickTime      = millis();
  lastPrintTime     = millis();
  windowStartTime   = millis();
  totalElapsed      = 0;
  lcdNeedsRedraw    = true;
  aiCorrectionInt   = 0.0;
  aiCorrection      = 0.0;

  Serial.print(F("[START] M")); Serial.print(selectedMode);
  Serial.print(F(" ")); Serial.print(getModeName());
  Serial.print(F(" WP")); Serial.print(waypointIndex);
  Serial.print(F(" target=")); Serial.println(Setpoint, 1);
}

void abortCycle() {
  setSSR(false); setFan(255);
  isRunning = false; fanForced = true;
  cycleState = IDLE; idleReady = false;
  integral = 0.0; Output = 0;
  aiCorrectionInt = 0.0; aiCorrection = 0.0;
  lcdNeedsRedraw = true;
  Serial.println(F("[ABORT] Aborted. Fan running."));
}

// ═════════════════════════════════════════════════════════════
//  ADVANCE WAYPOINT
// ═════════════════════════════════════════════════════════════
void advanceWaypoint(const char* reason) {
  waypointIndex++;
  integral          = 0.0;
  lastInput         = lastValidTemp;
  waypointStartTime = millis();
  soakCounter       = 0;
  lcdNeedsRedraw    = true;

  if (waypointIndex >= NUM_POINTS) {
    setSSR(false); setFan(255);
    fanForced  = true; isRunning = true;
    cycleState = COOLING_DOWN; Setpoint = 25.0;
    Serial.println(F("[COOL] Profile done. Fan ON."));
    return;
  }

  Setpoint   = waypointTemp[waypointIndex];
  cycleState = RAMPING;
  DLOG(Serial.print(F("[ADV] WP")); Serial.print(waypointIndex);
       Serial.print(F(" ")); Serial.print(waypointLabel[waypointIndex]);
       Serial.print(F(" -> ")); Serial.print(Setpoint, 1);
       Serial.print(F("C (")); Serial.print(reason); Serial.println(F(")")));
}

// ═════════════════════════════════════════════════════════════
//  TEMPERATURE GLITCH FILTER
// ═════════════════════════════════════════════════════════════
float readTempFiltered() {
  uint8_t fault = thermocouple.readError();
  if (fault) {
    glitchCounter++;
    if (glitchCounter >= GLITCH_MAX_CONSEC) return -1.0;
    return lastValidTemp;
  }
  float raw = thermocouple.readCelsius();
  if (isnan(raw))                          { if (++glitchCounter >= GLITCH_MAX_CONSEC) return -1.0; return lastValidTemp; }
  if (raw < GLITCH_ZERO_MAX)               { if (++glitchCounter >= GLITCH_MAX_CONSEC) return -1.0; return lastValidTemp; }
  float maxDelta = fanForced ? GLITCH_RATE_COOL : GLITCH_RATE_HEAT;
  if (abs(raw - lastValidTemp) > maxDelta) { if (++glitchCounter >= GLITCH_MAX_CONSEC) return -1.0; return lastValidTemp; }
  glitchCounter = 0;
  lastValidTemp = raw;
  return raw;
}

// ═════════════════════════════════════════════════════════════
//  SERIAL LOG
// ═════════════════════════════════════════════════════════════
void logSerial(float temp) {
  const char* phaseStr = "IDLE";
  if      (cycleState == RAMPING)      phaseStr = "RAMP";
  else if (cycleState == SOAKING)      phaseStr = "SOAK";
  else if (cycleState == COOLING_DOWN) phaseStr = "COOL";
  else if (cycleState == DONE)         phaseStr = "DONE";
  else if (cycleState == FAULT)        phaseStr = "FAULT";

  float ssrPct = (float)Output / WINDOW_MS * 100.0;

  Serial.print(F("Target:"));    Serial.print(Setpoint, 1);
  Serial.print(F(",Actual:"));   Serial.print(temp, 2);
  Serial.print(F(",Kp:"));       Serial.print(curKp, 0);
  Serial.print(F(",Ki:"));       Serial.print(curKi, 2);
  Serial.print(F(",Kd:"));       Serial.print(curKd, 0);
  Serial.print(F(",Out:"));      Serial.print(Output, 0);
  Serial.print(F(",AI_Active:")); Serial.print(aiActive ? 1 : 0);
  Serial.print(F(",AI_Corr:"));  Serial.print((int)aiCorrectionInt);
  Serial.print(F(",Phase:"));    Serial.print(phaseStr);
  Serial.print(F(",WP:"));       Serial.print(waypointIndex);
  Serial.print(F(",SSR:"));      Serial.print(ssrPct, 0);
  Serial.print(F(",t:"));        Serial.println(totalElapsed);
}

// ═════════════════════════════════════════════════════════════
//  ZONE PID — MODE-AWARE
// ═════════════════════════════════════════════════════════════
PIDGains getZoneGains(float temp, float setpoint) {
  float error = setpoint - temp;

  if (selectedMode == 3) {
    if (temp < 60.0)  return {100.0, 0.20, 8.0};
    if (temp < 100.0) return {140.0, 0.25, 12.0};
    if (setpoint <= 145.0 && temp >= 90.0) {
      return (abs(error) < 5.0) ? PIDGains{80.0,0.40,20.0} : PIDGains{110.0,0.50,25.0};
    }
    if (setpoint > 145.0) {
      return (temp > 160.0) ? PIDGains{60.0,0.30,35.0} : PIDGains{100.0,0.40,30.0};
    }
    return {120.0, 0.50, 25.0};
  }

  float soakThresh = (selectedMode == 2) ? 183.0 : 185.0;

  if (temp < 100.0) return {220.0, 0.20, 10.0};
  if (temp < 150.0) return (error > 20.0) ? PIDGains{180.0,0.30,15.0} : PIDGains{120.0,0.60,30.0};

  if (setpoint <= soakThresh && temp >= 140.0) {
    if (abs(error) <  5.0) return {100.0, 0.50, 40.0};
    if (abs(error) < 15.0) return {130.0, 0.40, 35.0};
    else                   return {160.0, 0.30, 25.0};
  }
  if (setpoint > soakThresh) {
    if (error > 30.0) return {260.0, 0.30,  8.0};
    if (error > 10.0) return {200.0, 0.50, 20.0};
    if (error >  0.0) return { 80.0, 0.70, 90.0};
    else              return { 40.0, 0.00,120.0};
  }
  return {150.0, 0.80, 30.0};
}

// ── HELPERS ───────────────────────────────────────────────────
void setSSR(bool on)   { digitalWrite(SSR_PIN, on ? HIGH : LOW); }
void setFan(int speed) { ledcWrite(FAN_LEDC_CH, constrain(speed, 0, 255)); }

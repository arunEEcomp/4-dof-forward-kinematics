/*
 * 4-DOF Robotic Arm Controller — v2.6-holdfix (17 Jul 2025)
 * − Fixed position holding when joystick is released
 * − Hysteretic dead band prevents jitter
 * − Exponential IIR filter for smooth ADC readings
 * − State machine approach for robust hold functionality
 * − Maintains 30 °/s speed control and full serial compatibility
 */

#include <Servo.h>

/* ---------------- user-tweakables ---------------- */
const int HOME[5]      = {90, 90, 180, 0, 0};
const int MIN_ANG[5]   = {0, 10, 10, 0, 0};
const int MAX_ANG[5]   = {180, 150, 180, 180, 85};
const int SPEED_DPS    = 30;          // target velocity degree/second
const int DBAND_INNER  = 80;          // inner hysteresis edge
const int DBAND_OUTER  = 120;         // outer hysteresis edge  
const float FILT_ALPHA = 0.25;        // IIR smoothing factor
const int HOLD_DEBOUNCE_MS = 50;      // minimum time before hold activation
/* ------------------------------------------------- */

const byte SIG_PIN[5]  = {12, 11, 10, 9, 8};
const byte X_PIN[3]    = {A0, A2, A4};
const byte Y_PIN[3]    = {A1, A3, A5};

// Joystick axis mapping: [base-X, shoulder-Y, elbow-X, wrist-Y, gripper-Y]
const byte JOY_PINS[5] = {A0, A1, A2, A3, A5};

enum JoystickState : uint8_t { 
  HOLD = 0, 
  ACTIVE = 1 
};

Servo srv[5];
int curDeg[5] = {HOME[0], HOME[1], HOME[2], HOME[3], HOME[4]};
int tgtDeg[5] = {HOME[0], HOME[1], HOME[2], HOME[3], HOME[4]};
int holdPosition[5] = {HOME[0], HOME[1], HOME[2], HOME[3], HOME[4]};
float filtADC[5] = {512, 512, 512, 512, 512};
JoystickState axisState[5] = {HOLD, HOLD, HOLD, HOLD, HOLD};
unsigned long lastActiveTime[5] = {0, 0, 0, 0, 0};
bool attached[5];
bool jsEnabled = false;

/* time-ramp helpers */
unsigned long lastStepMS[5];
const unsigned long STEP_INTERVAL_MS = 1000UL / SPEED_DPS;
const int ADC_CENTER = 512;

void setup() {
  Serial.begin(115200);
  for (byte i = 0; i < 5; i++) { 
    srv[i].attach(SIG_PIN[i]); 
    attached[i] = true;
    srv[i].write(curDeg[i]);
  }
  home(true);  // hard home on power-up
  Serial.println(F("READY"));
}

void loop() {
  pollSerial();
  pollJoysticks();
  timeRamp();
  streamAngles();
}

/* ---------- serial commands ---------- */
void pollSerial() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buf.trim(); 
      buf.toUpperCase();
      if (buf == "HOME") {
        home(false);
      }
      else if (buf == "ENABLE") {
        jsEnabled = true;
        // Set current positions as hold positions
        for (byte i = 0; i < 5; i++) {
          holdPosition[i] = curDeg[i];
          axisState[i] = HOLD;
        }
        Serial.println("OK");
      }
      else if (buf == "DISABLE") {
        jsEnabled = false; 
        Serial.println("OK");
      }
      else if (buf.startsWith("GRIP:")) {
        int g = constrain(buf.substring(5).toInt(), MIN_ANG[4], MAX_ANG[4]);
        tgtDeg[4] = g;
        holdPosition[4] = g;
      }
      else if (buf == "STATUS") {
        sendStatus();
      }
      else if (buf == "?ANGLES") {
        sendAngles();
      }
      buf = "";
    } else {
      buf += c;
    }
  }
}

/* ---------- improved joystick processing ---------- */
void pollJoysticks() {
  if (!jsEnabled) return;
  
  // Read and filter all joystick axes
  for (byte i = 0; i < 5; i++) {
    int raw = analogRead(JOY_PINS[i]);
    // Exponential moving average filter
    filtADC[i] = filtADC[i] + FILT_ALPHA * (raw - filtADC[i]);
    
    processJoystickAxis(i);
  }
}

/* ---------- process individual axis with hysteresis ---------- */
void processJoystickAxis(byte axisIndex) {
  int filteredValue = (int)filtADC[axisIndex];
  int deviation = abs(filteredValue - ADC_CENTER);
  unsigned long currentTime = millis();
  
  switch (axisState[axisIndex]) {
    case HOLD:
      // Check if joystick moved beyond outer threshold
      if (deviation > DBAND_OUTER) {
        axisState[axisIndex] = ACTIVE;
        lastActiveTime[axisIndex] = currentTime;
        // Immediately update target based on joystick position
        updateTargetFromJoystick(axisIndex, filteredValue);
      } else {
        // Stay in hold - maintain last hold position
        tgtDeg[axisIndex] = holdPosition[axisIndex];
      }
      break;
      
    case ACTIVE:
      if (deviation < DBAND_INNER) {
        // Check debounce time before switching to hold
        if (currentTime - lastActiveTime[axisIndex] > HOLD_DEBOUNCE_MS) {
          axisState[axisIndex] = HOLD;
          holdPosition[axisIndex] = curDeg[axisIndex]; // Lock current position
          tgtDeg[axisIndex] = holdPosition[axisIndex];
        }
      } else {
        // Continue active movement
        lastActiveTime[axisIndex] = currentTime;
        updateTargetFromJoystick(axisIndex, filteredValue);
      }
      break;
  }
}

/* ---------- update target based on joystick position ---------- */
void updateTargetFromJoystick(byte axisIndex, int adcValue) {
  int newTarget = map(adcValue, 0, 1023, MIN_ANG[axisIndex], MAX_ANG[axisIndex]);
  newTarget = constrain(newTarget, MIN_ANG[axisIndex], MAX_ANG[axisIndex]);
  tgtDeg[axisIndex] = newTarget;
}

/* ---------- time-based servo movement ---------- */
void timeRamp() {
  unsigned long now = millis();
  for (byte i = 0; i < 5; i++) {
    if (curDeg[i] == tgtDeg[i]) continue;
    if (now - lastStepMS[i] < STEP_INTERVAL_MS) continue;

    curDeg[i] += (tgtDeg[i] > curDeg[i]) ? 1 : -1;
    srv[i].write(curDeg[i]);
    lastStepMS[i] = now;
  }
}

/* ---------- helper functions ---------- */
void home(bool hard) {
  for (byte i = 0; i < 5; i++) {
    tgtDeg[i] = HOME[i];
    holdPosition[i] = HOME[i];
    axisState[i] = HOLD;
    if (hard) { 
      curDeg[i] = HOME[i]; 
      srv[i].write(curDeg[i]); 
    }
  }
}

void sendAngles() {
  Serial.print("ANGLES");
  for (byte i = 0; i < 5; i++) { 
    Serial.print(','); 
    Serial.print(curDeg[i]); 
  }
  Serial.println();
}

void sendStatus() {
  Serial.print("STATUS,");
  Serial.print(jsEnabled ? 1 : 0);
  for (byte i = 0; i < 5; i++) { 
    Serial.print(','); 
    Serial.print(attached[i] ? 1 : 0); 
  }
  Serial.println();
}

void streamAngles() {
  static unsigned long last = 0;
  if (millis() - last < 100) return;
  sendAngles();
  last = millis();
}

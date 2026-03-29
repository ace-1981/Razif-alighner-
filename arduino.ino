#include <Arduino.h>

// ======================= PINS =======================
// IBT-2 / H-Bridge PWM
const uint8_t PIN_PWM_RIGHT = 5;   // RPWM
const uint8_t PIN_PWM_LEFT  = 6;   // LPWM

// Mode input (24V->adapter to Arduino)
const uint8_t PIN_MODE_SEEK = 2;   // ACTIVE => homing (seek center), else => analog control

// Sensors (24V->adapter to Arduino)
const uint8_t PIN_CENTER_SENSOR = 3;   // inductive center sensor

// Limit switches (NC opens at limit)
const uint8_t PIN_LIMIT_LEFT  = 7;     // Left limit (NC)
const uint8_t PIN_LIMIT_RIGHT = 12;    // Right limit (NC)

// Analog edge/eye sensor
const uint8_t PIN_ANALOG_SENSOR = A0;  // 0..5V after divider/adapter

// ======================= POLARITY =======================
// Adjust to match your adapter logic
const bool MODE_SEEK_ACTIVE_HIGH = true;   // D2 HIGH => seek mode enabled
const bool CENTER_ACTIVE_HIGH    = false;  // D3 active LOW (your current wiring)
// NC + INPUT_PULLUP: normal LOW, at limit HIGH
const bool LIMITS_ARE_NC_PULLUP  = true;

// ======================= ANALOG CONTROL =======================
const float ANALOG_CENTER_V   = 2.00f;  // center setpoint
const float ANALOG_DEADBAND_V = 0.08f;
const float KP = 90.0f;

const int PWM_MIN = 55;
const int PWM_MAX = 200;

// Mapping: err>0 => move RIGHT?
bool ERR_POS_MOVE_RIGHT = true;

// ======================= JOG (serial override) =======================
// Send 'R' = jog right, 'L' = jog left, 'S' = stop jog
const int JOG_PWM = 60;
int8_t g_jog = 0;   // -1 left, 0 off, +1 right

void processSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if      (c == 'R') g_jog = +1;
    else if (c == 'L') g_jog = -1;
    else if (c == 'S') g_jog =  0;
  }
}

// ======================= LIMIT AUTO-BACKOFF =======================
// When motor first touches a limit, back off automatically so it never stays stuck
const int    LIMIT_BACKOFF_PWM = 55;
const uint16_t LIMIT_BACKOFF_MS  = 150;

bool prevLimLeft  = false;
bool prevLimRight = false;

void checkLimitBackoff() {
  bool limL = limitLeftActive();
  bool limR = limitRightActive();

  if (limL && !prevLimLeft) {      // just hit left limit
    motorStop();
    delay(20);
    motorRight(LIMIT_BACKOFF_PWM);
    while (limitLeftActive()) { delay(5); }  // reverse until off limit
    motorStop();
    seekDirRight = true;   // flip homing direction -> go right now
  }
  if (limR && !prevLimRight) {     // just hit right limit
    motorStop();
    delay(20);
    motorLeft(LIMIT_BACKOFF_PWM);
    while (limitRightActive()) { delay(5); }  // reverse until off limit
    motorStop();
    seekDirRight = false;  // flip homing direction -> go left now
  }

  prevLimLeft  = limL;
  prevLimRight = limR;
}

// ======================= HOMING (SEEK) =======================
// Fast search speed
const int SEEK_PWM_FAST = 18;
// Backoff speed (slow)
const int SEEK_PWM_SLOW = 10;
// Fine approach speed (very slow)
const int SEEK_PWM_FINE = 12;

const uint16_t BOUNCE_DELAY_MS = 80;

// Homing states:
// 1) Find center (fast) until CENTER becomes ON
// 2) Backoff (slow) until CENTER becomes OFF
// 3) Approach (fine) until CENTER becomes ON again
// 4) Done
enum HomingState : uint8_t {
  SEEK_FIND_CENTER_FAST = 0,
  SEEK_BACKOFF_UNTIL_OFF = 1,
  SEEK_APPROACH_FINE = 2,
  SEEK_DONE = 3
};

HomingState homingState = SEEK_FIND_CENTER_FAST;

// Seek direction memory: true => right, false => left
bool seekDirRight = true;

// ======================= MOTOR TRACKING =======================
int g_dir = 0;  // -1 left, 0 stop, +1 right
int g_pwm = 0;  // 0..255

static inline void motorStop() {
  g_dir = 0; g_pwm = 0;
  analogWrite(PIN_PWM_RIGHT, 0);
  analogWrite(PIN_PWM_LEFT,  0);
}
static inline void motorRight(int pwm) {
  pwm = constrain(pwm, 0, 255);
  g_dir = +1; g_pwm = pwm;
  analogWrite(PIN_PWM_RIGHT, pwm);
  analogWrite(PIN_PWM_LEFT,  0);
}
static inline void motorLeft(int pwm) {
  pwm = constrain(pwm, 0, 255);
  g_dir = -1; g_pwm = pwm;
  analogWrite(PIN_PWM_RIGHT, 0);
  analogWrite(PIN_PWM_LEFT,  pwm);
}

// ======================= INPUT HELPERS =======================
static inline bool isActiveDigital(uint8_t pin, bool activeHigh) {
  int raw = digitalRead(pin);
  return activeHigh ? (raw == HIGH) : (raw == LOW);
}

bool seekModeEnabled() {
  return isActiveDigital(PIN_MODE_SEEK, MODE_SEEK_ACTIVE_HIGH);
}

bool centerSensorActive() {
  return isActiveDigital(PIN_CENTER_SENSOR, CENTER_ACTIVE_HIGH);
}

// NC + INPUT_PULLUP: normal LOW, at limit HIGH => active HIGH
bool limitLeftActive() {
  int raw = digitalRead(PIN_LIMIT_LEFT);
  return LIMITS_ARE_NC_PULLUP ? (raw == HIGH) : (raw == LOW);
}
bool limitRightActive() {
  int raw = digitalRead(PIN_LIMIT_RIGHT);
  return LIMITS_ARE_NC_PULLUP ? (raw == HIGH) : (raw == LOW);
}

// ======================= ANALOG SENSOR =======================
int readAnalogRawAvg(uint8_t n=10) {
  long s = 0;
  for (uint8_t i=0; i<n; i++) {
    s += analogRead(PIN_ANALOG_SENSOR);
    delayMicroseconds(300);
  }
  return (int)(s/n);
}
float rawToV(int raw) {
  return (raw * 5.0f) / 1023.0f;
}

// ======================= HOMING CONTROL =======================
void homingReset() {
  homingState = SEEK_FIND_CENTER_FAST;
  seekDirRight = true;       // start moving right first
  motorStop();
}

void homingStep() {
  bool cen = centerSensorActive();

  // Safety: impossible that both limits are active -> stop
  if (limitLeftActive() && limitRightActive()) { motorStop(); return; }

  switch (homingState) {

    case SEEK_FIND_CENTER_FAST: {
      // If center is ON -> stop, go to backoff phase
      if (cen) {
        motorStop();
        homingState = SEEK_BACKOFF_UNTIL_OFF;
        return;
      }

      // Move fast in seekDir (limits handled by checkLimitBackoff)
      if (seekDirRight) {
        if (!limitRightActive()) motorRight(SEEK_PWM_FAST);
        else motorStop();
      } else {
        if (!limitLeftActive()) motorLeft(SEEK_PWM_FAST);
        else motorStop();
      }
      return;
    }

    case SEEK_BACKOFF_UNTIL_OFF: {
      // Move opposite direction SLOW until center becomes OFF
      if (!cen) {
        motorStop();
        homingState = SEEK_APPROACH_FINE;
        return;
      }

      // backoff opposite seekDir
      if (seekDirRight) { // seekDir was right => backoff left
        if (!limitLeftActive()) motorLeft(SEEK_PWM_SLOW);
        else if (!limitRightActive()) motorRight(SEEK_PWM_SLOW);
        else motorStop();
      } else {            // seekDir was left => backoff right
        if (!limitRightActive()) motorRight(SEEK_PWM_SLOW);
        else if (!limitLeftActive()) motorLeft(SEEK_PWM_SLOW);
        else motorStop();
      }
      return;
    }

    case SEEK_APPROACH_FINE: {
      // Now approach again VERY SLOW in seekDir until center becomes ON
      if (cen) {
        motorStop();
        homingState = SEEK_DONE;
        return;
      }

      // Safety (limits handled by checkLimitBackoff)

      if (seekDirRight) {
        if (!limitRightActive()) motorRight(SEEK_PWM_FINE);
        else motorStop();
      } else {
        if (!limitLeftActive()) motorLeft(SEEK_PWM_FINE);
        else motorStop();
      }
      return;
    }

    case SEEK_DONE:
    default:
      motorStop();
      return;
  }
}

// ======================= ANALOG CONTROL =======================
void analogControlStep() {
  float v = rawToV(readAnalogRawAvg(10));
  float err = v - ANALOG_CENTER_V;

  if (fabs(err) <= ANALOG_DEADBAND_V) {
    motorStop();
    return;
  }

  int pwm = (int)(PWM_MIN + KP * fabs(err));
  if (pwm > PWM_MAX) pwm = PWM_MAX;

  bool moveRight = (err > 0) ? ERR_POS_MOVE_RIGHT : !ERR_POS_MOVE_RIGHT;

  if (moveRight) {
    if (limitRightActive()) { motorStop(); return; }
    motorRight(pwm);
  } else {
    if (limitLeftActive()) { motorStop(); return; }
    motorLeft(pwm);
  }
}

// ======================= TELEMETRY =======================
void telemetryLine() {
  // raw
  int rawD2 = digitalRead(PIN_MODE_SEEK);
  int rawD3 = digitalRead(PIN_CENTER_SENSOR);
  int rawLL = digitalRead(PIN_LIMIT_LEFT);
  int rawLR = digitalRead(PIN_LIMIT_RIGHT);

  // active logic
  bool seekEn = seekModeEnabled();
  bool cenAct = centerSensorActive();
  bool limL   = limitLeftActive();
  bool limR   = limitRightActive();

  float v = rawToV(readAnalogRawAvg(6));

  Serial.print("T ");
  Serial.print("seek=");   Serial.print(seekEn ? 1 : 0);
  Serial.print(" cen=");   Serial.print(cenAct ? 1 : 0);
  Serial.print(" limL=");  Serial.print(limL ? 1 : 0);
  Serial.print(" limR=");  Serial.print(limR ? 1 : 0);

  Serial.print(" rawD2="); Serial.print(rawD2);
  Serial.print(" rawD3="); Serial.print(rawD3);
  Serial.print(" rawLL="); Serial.print(rawLL);
  Serial.print(" rawLR="); Serial.print(rawLR);

  Serial.print(" aV=");    Serial.print(v, 3);

  Serial.print(" dir=");   Serial.print(g_dir);
  Serial.print(" pwm=");   Serial.print(g_pwm);

  Serial.print(" hs=");    Serial.print((int)homingState);
  Serial.print(" seekDir="); Serial.print(seekDirRight ? 1 : 0); // 1=right,0=left
  Serial.println();
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RENESAS) || defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  analogReadResolution(10);
#endif

  pinMode(PIN_PWM_RIGHT, OUTPUT);
  pinMode(PIN_PWM_LEFT,  OUTPUT);

  pinMode(PIN_MODE_SEEK,     INPUT_PULLUP);
  pinMode(PIN_CENTER_SENSOR, INPUT_PULLUP);

  pinMode(PIN_LIMIT_LEFT,  INPUT_PULLUP);
  pinMode(PIN_LIMIT_RIGHT, INPUT_PULLUP);

  motorStop();
  homingReset();
  Serial.println("=== START align_clean_telemetry ===");
}

void loop() {
  processSerialCommands();
  checkLimitBackoff();

  if (g_jog != 0) {
    // jog override: move motor directly, ignore seek/analog
    if (g_jog > 0) {
      if (!limitRightActive()) motorRight(JOG_PWM);
      else motorStop();
    } else {
      if (!limitLeftActive()) motorLeft(JOG_PWM);
      else motorStop();
    }
  } else if (seekModeEnabled()) {
    // homing mode
    if (homingState == SEEK_DONE) motorStop();
    else homingStep();
  } else {
    // normal analog mode
    // reset homing so next seek starts clean
    if (homingState != SEEK_FIND_CENTER_FAST) homingReset();
    analogControlStep();
  }

  static unsigned long t=0;
  if (millis() - t > 100) { t = millis(); telemetryLine(); }

  delay(5);
}

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
const bool CENTER_ACTIVE_HIGH    = true;   // D3 active HIGH (adapter pulls HIGH when detecting)
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

// Seek direction memory: true => right, false => left
// DEFAULT: start LEFT (towards center sensor - confirmed)
bool seekDirRight = false;

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
// If ANY limit is active, reverse current motor direction until limit clears.
// This works regardless of wiring polarity.
const int LIMIT_BACKOFF_PWM = 55;

void checkLimitBackoff() {
  bool limL = limitLeftActive();
  bool limR = limitRightActive();
  if (!limL && !limR) return;

  int wasDir = g_dir;  // -1 left, 0 stop, +1 right
  motorStop();
  delay(20);

  // Decide escape direction:
  // If we know which limit is hit, run AWAY from it.
  // If both limits (error) or motor was stopped, use limit info.
  int escapeDir;
  if (limL && !limR) {
    escapeDir = +1;  // left limit -> escape right
  } else if (limR && !limL) {
    escapeDir = -1;  // right limit -> escape left
  } else if (wasDir != 0) {
    escapeDir = -wasDir;  // reverse previous direction
  } else {
    escapeDir = +1;  // fallback: escape right
  }

  if (escapeDir > 0) {
    motorRight(LIMIT_BACKOFF_PWM);
  } else {
    motorLeft(LIMIT_BACKOFF_PWM);
  }

  // Keep reversing until ALL limits clear
  while (limitLeftActive() || limitRightActive()) { delay(5); }
  motorStop();
}

// ======================= HOMING (TIMING-BASED) =======================
// Calibration speed - must be enough to overcome motor friction
const int CAL_PWM = 65;
const int SETTLE_DELAY_MS = 200;     // pause at limits during calibration
const float SAFETY_MARGIN = 1.15f;   // 15% extra on max travel time

// Homing states:
// 1) Move left until left limit
// 2) Move right until right limit (measure travel time)
// 3) Move left for half the travel time (center)
// 4) Done
enum HomingState : uint8_t {
  CAL_FIND_LEFT_LIMIT = 0,
  CAL_FIND_RIGHT_LIMIT = 1,
  CAL_GO_TO_CENTER = 2,
  SEEK_DONE = 3
};

HomingState homingState = CAL_FIND_LEFT_LIMIT;

// Calibration data
unsigned long calStartMs = 0;        // timer for current phase
unsigned long calTravelMs = 0;       // measured full travel time (limit to limit)
unsigned long calHalfTravelMs = 0;   // half = center
unsigned long maxRunMs = 0;          // safety: max allowed motor run per direction
bool calibrated = false;

// Motor runtime safety
unsigned long motorRunStartMs = 0;
int lastMotorDir = 0;

// ======================= MOTOR TRACKING =======================
// (g_dir, g_pwm, motorStop/Right/Left, seekDirRight, input helpers defined above)

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

// ======================= HOMING CONTROL (TIMING) =======================
void homingReset() {
  homingState = CAL_FIND_LEFT_LIMIT;
  calStartMs = 0;
  motorStop();
}

void homingStep() {
  // Safety: both limits active = error
  if (limitLeftActive() && limitRightActive()) { motorStop(); return; }

  switch (homingState) {

    case CAL_FIND_LEFT_LIMIT: {
      // Move left until left limit
      if (limitLeftActive()) {
        motorStop();
        delay(SETTLE_DELAY_MS);
        calStartMs = millis();   // start timing the rightward trip
        homingState = CAL_FIND_RIGHT_LIMIT;
        return;
      }
      motorLeft(CAL_PWM);
      return;
    }

    case CAL_FIND_RIGHT_LIMIT: {
      // Move right, measuring time until right limit
      if (limitRightActive()) {
        calTravelMs = millis() - calStartMs;
        calHalfTravelMs = calTravelMs / 2;
        maxRunMs = (unsigned long)(calTravelMs * SAFETY_MARGIN);
        calibrated = true;
        motorStop();
        delay(SETTLE_DELAY_MS);
        calStartMs = millis();   // start timing the center-bound trip
        homingState = CAL_GO_TO_CENTER;
        return;
      }
      motorRight(CAL_PWM);
      return;
    }

    case CAL_GO_TO_CENTER: {
      // Move left for half the measured travel time
      unsigned long elapsed = millis() - calStartMs;
      if (elapsed >= calHalfTravelMs) {
        motorStop();
        homingState = SEEK_DONE;
        return;
      }
      motorLeft(CAL_PWM);
      return;
    }

    case SEEK_DONE:
    default:
      motorStop();
      return;
  }
}

// ======================= MOTOR SAFETY TIMEOUT =======================
void checkMotorSafetyTimeout() {
  if (!calibrated || maxRunMs == 0) return;
  // Don't limit during active calibration
  if (seekModeEnabled() && homingState != SEEK_DONE) return;

  if (g_dir != 0) {
    if (g_dir != lastMotorDir) {
      motorRunStartMs = millis();
      lastMotorDir = g_dir;
    } else if (millis() - motorRunStartMs > maxRunMs) {
      motorStop();  // exceeded max travel time - limit switch may have failed
    }
  } else {
    lastMotorDir = 0;
    motorRunStartMs = millis();
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
  Serial.print(" cal=");   Serial.print(calibrated ? 1 : 0);
  Serial.print(" trvMs="); Serial.print(calTravelMs);
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
  pinMode(PIN_CENTER_SENSOR, INPUT);        // no pullup - let adapter drive D3

  pinMode(PIN_LIMIT_LEFT,  INPUT_PULLUP);
  pinMode(PIN_LIMIT_RIGHT, INPUT_PULLUP);

  motorStop();
  homingReset();
  Serial.println("=== START align_clean_telemetry ===");
  
  // Startup diagnostics
  Serial.print("DIAG: rawLL="); Serial.print(digitalRead(PIN_LIMIT_LEFT));
  Serial.print(" rawLR="); Serial.print(digitalRead(PIN_LIMIT_RIGHT));
  Serial.print(" limL="); Serial.print(limitLeftActive() ? 1 : 0);
  Serial.print(" limR="); Serial.print(limitRightActive() ? 1 : 0);
  Serial.println();
}

void loop() {
  processSerialCommands();

  static bool wasSeekMode = false;
  bool isSeek = seekModeEnabled();

  if (g_jog != 0) {
    // jog override: limit backoff active for safety
    checkLimitBackoff();
    if (g_jog > 0) {
      if (!limitRightActive()) motorRight(JOG_PWM);
      else motorStop();
    } else {
      if (!limitLeftActive()) motorLeft(JOG_PWM);
      else motorStop();
    }
    wasSeekMode = false;
  } else if (isSeek) {
    // homing mode - limits handled inside homingStep (flip direction)
    // NO checkLimitBackoff here - homing manages its own limits
    if (!wasSeekMode) {
      homingReset();
      wasSeekMode = true;
    }
    if (homingState == SEEK_DONE) motorStop();
    else homingStep();
  } else {
    // normal analog mode - limit backoff active for safety
    checkLimitBackoff();
    if (wasSeekMode) homingReset();
    wasSeekMode = false;
    analogControlStep();
  }

  checkMotorSafetyTimeout();

  static unsigned long t=0;
  if (millis() - t > 100) { t = millis(); telemetryLine(); }

  delay(5);
}

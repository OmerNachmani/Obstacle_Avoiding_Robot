// --- Fix Arduino auto-prototype issue: make DCMotor known very early ---
struct DCMotor;

#include <Arduino.h>
#include <Servo.h>

/* ======== Display type ======== */
const bool COMMON_CATHODE = true;

/* ===== Pins (Arduino DUE) ===== */
const uint8_t TRIG_PIN  = 2;    // HC-SR04 TRIG (3.3V OK)
const uint8_t ECHO_PIN  = 12;   // HC-SR04 ECHO -> ***LEVEL SHIFT TO 3.3V***
const uint8_t SERVO_PIN = 3;

const uint8_t SEG_A = 4, SEG_B = 5, SEG_C = 6, SEG_D = 7,
              SEG_E = 8, SEG_F = 9, SEG_G = 10;
const uint8_t DIG1 = 13, DIG2 = 22, DIG3 = 11; // common-cathode digit enables

const uint8_t LED_R = 23, LED_Y = 24, LED_G = 25;
const uint8_t BUZZ  = 26;      // Active buzzer assumed (on/off)

/* ===== L293D DC motors =====
   Motor1 header: 1=EN -> D33, 2=IN1 -> D32, 7=IN2 -> D31
   Motor2 header: 1=EN -> D27, 2=IN1 -> D28, 7=IN2 -> D29
*/
const uint8_t M1_EN  = 33;
const uint8_t M1_IN1 = 32;
const uint8_t M1_IN2 = 31;

const uint8_t M2_EN  = 27;
const uint8_t M2_IN1 = 28;
const uint8_t M2_IN2 = 29;

/* ===== Behavior ===== */
const int DIST_CLOSE_CM  = 30;   // <30 -> red + buzzer
const int DIST_WARN_CM   = 70;   // 30..70 -> yellow only
const int MAX_DISPLAY_CM = 999;

// Narrow sweep
const int SERVO_LEFT  = 20;
const int SERVO_RIGHT = 110;
const int SERVO_STEP  = 1;
const int SERVO_SETTLE_MS = 15;

/* ===== Display refresh (software-timed, non-blocking) ===== */
const uint32_t DIGIT_PERIOD_US = 500;  // ~666 Hz per digit

const uint8_t SEG_PINS[7] = { SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G };
const uint8_t DIG_PINS[3] = { DIG1, DIG2, DIG3 };

// patterns: a,b,c,d,e,f,g  (1 = ON for common-cathode logic)
const uint8_t DIG_PATTERNS[10][7] = {
  /* 0 */ {1,1,1,1,1,1,0},
  /* 1 */ {0,1,1,0,0,0,0},
  /* 2 */ {1,1,0,1,1,0,1},
  /* 3 */ {1,1,1,1,0,0,1},
  /* 4 */ {0,1,1,0,0,1,1},
  /* 5 */ {1,0,1,1,0,1,1},
  /* 6 */ {1,0,1,1,1,1,1},
  /* 7 */ {1,1,1,0,0,0,0},
  /* 8 */ {1,1,1,1,1,1,1},
  /* 9 */ {1,1,1,1,0,1,1}
};

volatile int displayValue = 0;     // what we actually show
int pendingDisplayValue = 0;       // next value to push
uint8_t curDigit = 0;
uint32_t nextDigitAt = 0;
unsigned long nextDisplayAt = 0;

inline void allDigitsOff() {
  const bool digOff = COMMON_CATHODE ? HIGH : LOW;
  digitalWrite(DIG1, digOff);
  digitalWrite(DIG2, digOff);
  digitalWrite(DIG3, digOff);
}

inline void setSegmentsForValue(uint8_t value) {
  const uint8_t* pat = DIG_PATTERNS[value];
  const bool segOn  = COMMON_CATHODE ? HIGH : LOW;
  const bool segOff = COMMON_CATHODE ? LOW  : HIGH;
  for (uint8_t s = 0; s < 7; s++) digitalWrite(SEG_PINS[s], pat[s] ? segOn : segOff);
}

void refreshDisplaySoft() {
  const uint32_t now = micros();
  if ((int32_t)(now - nextDigitAt) < 0) return;
  nextDigitAt = now + DIGIT_PERIOD_US;

  int v = displayValue;
  if (v < 0) v = 0;
  if (v > MAX_DISPLAY_CM) v = MAX_DISPLAY_CM;

  const uint8_t ones =  v % 10;
  const uint8_t tens = (v / 10) % 10;
  const uint8_t hund = (v / 100) % 10;

  allDigitsOff();

  switch (curDigit) {
    case 0: setSegmentsForValue(hund); digitalWrite(DIG1, COMMON_CATHODE ? LOW : HIGH); break;
    case 1: setSegmentsForValue(tens); digitalWrite(DIG2, COMMON_CATHODE ? LOW : HIGH); break;
    case 2: setSegmentsForValue(ones); digitalWrite(DIG3, COMMON_CATHODE ? LOW : HIGH); break;
  }
  curDigit = (curDigit + 1) % 3;
}

/* ===== LEDs & buzzer ===== */
inline void setLEDs(bool r, bool y, bool g) {
  digitalWrite(LED_R, r?HIGH:LOW);
  digitalWrite(LED_Y, y?HIGH:LOW);
  digitalWrite(LED_G, g?HIGH:LOW);
}
inline void buzzerOn()  { digitalWrite(BUZZ, HIGH); }
inline void buzzerOff() { digitalWrite(BUZZ, LOW);  }

/* ===== Ultrasonic — interrupt-driven (non-blocking) ===== */
volatile uint32_t echoStart = 0, echoEnd = 0;
volatile bool echoHigh = false, echoReady = false;

void IR_echoChange() {
  if (digitalRead(ECHO_PIN)) { // Rising edge
    echoHigh = true; echoReady = false; echoStart = micros();
  } else if (echoHigh) {       // Falling edge
    echoEnd = micros(); echoHigh = false; echoReady = true;
  }
}

unsigned int distBuf[3] = {999,999,999};
uint8_t distIdx = 0;

unsigned int latestDistanceCM() {
  static unsigned int median = 999;
  if (echoReady) {
    noInterrupts();
    uint32_t start = echoStart, end = echoEnd;
    echoReady = false;
    interrupts();

    uint32_t dur = (end >= start) ? (end - start) : (0xFFFFFFFFu - start + end + 1u);
    distBuf[distIdx] = (dur > 30000UL) ? 999 : (unsigned int)(dur / 58UL);
    distIdx = (distIdx + 1) % 3;

    unsigned int a = distBuf[0], b = distBuf[1], c = distBuf[2];
    unsigned int hi = max(a, max(b, c)), lo = min(a, min(b, c));
    median = a + b + c - hi - lo;
  }
  return median;
}

void triggerPing() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
}

/* ===== Servo sweep ===== */
Servo head;
int servoAngle = SERVO_LEFT;
int servoDir   = +1;

/* ===== DC Motor driver (non-blocking software PWM on EN) ===== */
const uint32_t PWM_PERIOD_US = 5000;       // 200 Hz
const uint8_t  SPEED_FAST    = 255;        // duty 0..255
const uint8_t  SPEED_SLOW    = 120;

struct DCMotor {
  uint8_t en, in1, in2;
  int8_t  dir;           // +1 forward, -1 reverse
  bool    enabled;
  uint8_t duty;          // 0..255
  bool    pwmHigh;       // current EN state
  uint32_t nextToggleAt;
  uint32_t onTimeUs;
  uint32_t offTimeUs;
};

DCMotor m1{M1_EN, M1_IN1, M1_IN2, +1, true, SPEED_FAST, false, 0, 0, 0};
DCMotor m2{M2_EN, M2_IN1, M2_IN2, +1, true, SPEED_FAST, false, 0, 0, 0};

inline void dcApplyDir(const DCMotor& m) {
  if (m.dir >= 0) { digitalWrite(m.in1, HIGH); digitalWrite(m.in2, LOW);  }
  else            { digitalWrite(m.in1, LOW);  digitalWrite(m.in2, HIGH); }
}

inline void dcComputeTimes(DCMotor& m) {
  if (m.duty >= 255)      { m.onTimeUs = PWM_PERIOD_US; m.offTimeUs = 0; }
  else if (m.duty == 0)   { m.onTimeUs = 0;             m.offTimeUs = PWM_PERIOD_US; }
  else {
    m.onTimeUs  = (uint32_t)((uint64_t)PWM_PERIOD_US * m.duty / 255u);
    m.offTimeUs = PWM_PERIOD_US - m.onTimeUs;
  }
}

void dcInit(DCMotor& m) {
  pinMode(m.en,  OUTPUT);
  pinMode(m.in1, OUTPUT);
  pinMode(m.in2, OUTPUT);
  digitalWrite(m.en, LOW);
  dcApplyDir(m);
  dcComputeTimes(m);
  m.pwmHigh = false;
  m.nextToggleAt = micros() + 500;
}

inline void dcSetDir(DCMotor& m, int8_t dir) { m.dir = (dir >= 0) ? +1 : -1; dcApplyDir(m); }
inline void dcSetDuty(DCMotor& m, uint8_t duty) { m.duty = duty; dcComputeTimes(m); }
inline void dcEnable(DCMotor& m, bool on) { m.enabled = on; if (!on) digitalWrite(m.en, LOW); }

void dcUpdate(DCMotor& m) {
  if (!m.enabled) { digitalWrite(m.en, LOW); return; }
  if (m.onTimeUs == PWM_PERIOD_US) { digitalWrite(m.en, HIGH); return; } // 100%
  if (m.offTimeUs == PWM_PERIOD_US){ digitalWrite(m.en, LOW);  return; } // 0%

  const uint32_t now = micros();
  if ((int32_t)(now - m.nextToggleAt) < 0) return;

  if (m.pwmHigh) { digitalWrite(m.en, LOW);  m.pwmHigh = false; m.nextToggleAt = now + m.offTimeUs; }
  else           { digitalWrite(m.en, HIGH); m.pwmHigh = true;  m.nextToggleAt = now + m.onTimeUs;  }
}

/* ===== Control tiers for DC speed ===== */
enum Zone { Z_GREEN, Z_YELLOW, Z_RED };
Zone lastZone = Z_GREEN;
int stoppedMotor = -1; // 0=m1, 1=m2

/* ===== Setup ===== */
void setup() {
  // IO
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);      // ***ECHO must be level-shifted to 3.3V***
  for (uint8_t i=0;i<7;i++) pinMode(SEG_PINS[i], OUTPUT);
  for (uint8_t i=0;i<3;i++) pinMode(DIG_PINS[i], OUTPUT);
  pinMode(LED_R, OUTPUT); pinMode(LED_Y, OUTPUT); pinMode(LED_G, OUTPUT);
  pinMode(BUZZ, OUTPUT);

  // DC motors
  dcInit(m1);
  dcInit(m2);
  dcSetDir(m1, +1);  // flip to -1 if needed
  dcSetDir(m2, +1);
  dcSetDuty(m1, SPEED_FAST);
  dcSetDuty(m2, SPEED_FAST);
  dcEnable(m1, true);
  dcEnable(m2, true);

  // Seed randomness for red-zone choice
  randomSeed( (uint32_t) (analogRead(A0) + micros()) );

  // Display
  allDigitsOff();

  // Servo
  head.attach(SERVO_PIN);
  head.write(servoAngle);

  // LEDs + buzzer
  setLEDs(false,false,true);
  buzzerOff();

  // Ultrasonic interrupt
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), IR_echoChange, CHANGE);

  delay(50);
  triggerPing();

  nextDisplayAt = millis() + 10;
  nextDigitAt = micros() + DIGIT_PERIOD_US;
}

/* ===== Loop ===== */
void loop() {
  refreshDisplaySoft();

  // Servo sweep
  static unsigned long nextMoveAt = 0;
  if ((long)(millis() - nextMoveAt) >= 0) {
    nextMoveAt = millis() + SERVO_SETTLE_MS;
    servoAngle += servoDir * SERVO_STEP;
    if (servoAngle >= SERVO_RIGHT) { servoAngle = SERVO_RIGHT; servoDir = -1; }
    if (servoAngle <= SERVO_LEFT)  { servoAngle = SERVO_LEFT;  servoDir = +1; }
    head.write(servoAngle);
  }

  // Ultrasonic ~25 Hz
  static unsigned long nextPingAt = 0;
  if ((long)(millis() - nextPingAt) >= 0) { nextPingAt = millis() + 40; triggerPing(); }

  unsigned int cm = latestDistanceCM();

  // Zone
  Zone z;
  if (cm > (unsigned)DIST_WARN_CM)      z = Z_GREEN;
  else if (cm > (unsigned)DIST_CLOSE_CM)z = Z_YELLOW;
  else                                   z = Z_RED;

  // React to zone changes
  static Zone prev = Z_GREEN;
  if (z != prev) {
    switch (z) {
      case Z_GREEN:
        dcSetDuty(m1, SPEED_FAST); dcSetDuty(m2, SPEED_FAST);
        dcEnable(m1, true); dcEnable(m2, true);
        stoppedMotor = -1;
        break;
      case Z_YELLOW:
        dcSetDuty(m1, SPEED_SLOW); dcSetDuty(m2, SPEED_SLOW);
        dcEnable(m1, true); dcEnable(m2, true);
        stoppedMotor = -1;
        break;
      case Z_RED:
        stoppedMotor = random(0, 2); // 0=m1, 1=m2
        dcSetDuty(m1, SPEED_SLOW); dcSetDuty(m2, SPEED_SLOW);
        dcEnable(m1, stoppedMotor != 0);
        dcEnable(m2, stoppedMotor != 1);
        break;
    }
    prev = z;
  }

  // LEDs / buzzer / display
  if (z == Z_GREEN) {
    setLEDs(false,false,true);  pendingDisplayValue = (cm <= MAX_DISPLAY_CM) ? (int)cm : MAX_DISPLAY_CM; buzzerOff();
  } else if (z == Z_YELLOW) {
    setLEDs(false,true,false);  pendingDisplayValue = (cm <= MAX_DISPLAY_CM) ? (int)cm : MAX_DISPLAY_CM; buzzerOff();
  } else {
    setLEDs(true,false,false);  pendingDisplayValue = (cm <= MAX_DISPLAY_CM) ? (int)cm : MAX_DISPLAY_CM; buzzerOn();
  }

  if ((long)(millis() - nextDisplayAt) >= 0) {
    nextDisplayAt = millis() + 10;
    displayValue = pendingDisplayValue;
  }

  // Update DC PWM
  dcUpdate(m1);
  dcUpdate(m2);
}

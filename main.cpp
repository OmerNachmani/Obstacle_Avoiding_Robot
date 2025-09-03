struct DCMotor; // Forward declaration so functions/struct references compile even if definition appears later.

#include <Arduino.h> // Core Arduino APIs (pinMode, digitalWrite, millis, micros, etc.)
#include <Servo.h>   // Servo control library

/* ======== Display type ======== */
// 3-digit 7-seg is wired as common-cathode (CC). For CC: segment ON => drive pin HIGH; digit ENABLE => drive digit pin LOW.
const bool COMMON_CATHODE = true;

/* ===== Pins (Arduino DUE) ===== */
// Ultrasonic HC-SR04 pins (NOTE: DUE is 3.3V; ECHO must be level-shifted to 3.3V)
const uint8_t TRIG_PIN  = 2;    // Output: 10µs HIGH pulse fires a ping
const uint8_t ECHO_PIN  = 12;   // Input: echo pulse width encodes distance (needs level shifter on DUE)
const uint8_t SERVO_PIN = 3;    // Servo signal pin for the scanning head

// 7-seg segment pins (segments a..g). These drive the segment LEDs for ALL digits.
const uint8_t SEG_A = 4, SEG_B = 5, SEG_C = 6, SEG_D = 7,
              SEG_E = 8, SEG_F = 9, SEG_G = 10;

// Digit enable pins for the 3-digit CC display. Multiplexed one at a time.
const uint8_t DIG1 = 13, DIG2 = 22, DIG3 = 11; // Enable one digit LOW at a time (for CC)

// Status LEDs and buzzer (active buzzer: HIGH = sound)
const uint8_t LED_R = 23, LED_Y = 24, LED_G = 25;
const uint8_t BUZZ  = 26;      // Buzzer pin (no tone generation needed for active buzzer)

/* ===== L293D DC motors =====
   Each motor uses 3 control lines: EN (speed via PWM), IN1 and IN2 (direction).
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
// Distance thresholds (centimeters) for behavior tiers.
//   < 30cm  => RED (too close): evasive action + buzzer
//   30..70  => YELLOW (caution): slow forward
//   > 70    => GREEN (clear): fast forward
const int DIST_CLOSE_CM  = 30;
const int DIST_WARN_CM   = 70;
const int MAX_DISPLAY_CM = 999; // 3-digit display cap

// Servo sweep limits and pacing (degrees). Narrow field for scanning.
const int SERVO_LEFT  = 20;
const int SERVO_RIGHT = 110;
const int SERVO_STEP  = 1;     // 1° per step for smoothness
const int SERVO_SETTLE_MS = 15;// wait between steps for servo to settle

/* ===== Display refresh (software-timed, non-blocking) ===== */
// Time slice per digit in microseconds for multiplexing; ~666Hz per digit (3 digits => ~222Hz overall frame)
const uint32_t DIGIT_PERIOD_US = 500;

const uint8_t SEG_PINS[7] = { SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G };
const uint8_t DIG_PINS[3] = { DIG1, DIG2, DIG3 };

// Digit segment patterns for 0..9: order is a,b,c,d,e,f,g. For CC: 1=ON(HIGH), 0=OFF(LOW).
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

// Live value on display vs. pending (updated at 100Hz to avoid jitter). curDigit cycles 0..2 for multiplexing.
volatile int displayValue = 0;     // Value currently shown (0..999)
int pendingDisplayValue = 0;       // Next value to publish to display
uint8_t curDigit = 0;              // Which digit is active this slice
uint32_t nextDigitAt = 0;          // Next micros() for digit switch
unsigned long nextDisplayAt = 0;   // Next millis() to apply pending->displayValue

// Helper: disable all digits to avoid ghosting during segment updates.
inline void allDigitsOff() {
  const bool digOff = COMMON_CATHODE ? HIGH : LOW; // CC: HIGH disables digit
  digitalWrite(DIG1, digOff);
  digitalWrite(DIG2, digOff);
  digitalWrite(DIG3, digOff);
}

// Helper: push the 7-seg pattern for a single numeral onto segment pins.
inline void setSegmentsForValue(uint8_t value) {
  const uint8_t* pat = DIG_PATTERNS[value];
  const bool segOn  = COMMON_CATHODE ? HIGH : LOW; // CC: HIGH lights segment
  const bool segOff = COMMON_CATHODE ? LOW  : HIGH;
  for (uint8_t s = 0; s < 7; s++) digitalWrite(SEG_PINS[s], pat[s] ? segOn : segOff);
}

// Non-blocking display refresh: called often; advances a single digit each time slice.
void refreshDisplaySoft() {
  const uint32_t now = micros();
  if ((int32_t)(now - nextDigitAt) < 0) return; // Not time yet for next digit slice
  nextDigitAt = now + DIGIT_PERIOD_US;

  int v = displayValue;                // Bound the shown value into 0..999
  if (v < 0) v = 0;
  if (v > MAX_DISPLAY_CM) v = MAX_DISPLAY_CM;

  // Decompose into hundreds/tens/ones
  const uint8_t ones =  v % 10;
  const uint8_t tens = (v / 10) % 10;
  const uint8_t hund = (v / 100) % 10;

  allDigitsOff(); // Prevent ghosting before switching segments & enabling a digit

  // Enable exactly one digit (LOW for CC) after loading its segments
  switch (curDigit) {
    case 0: setSegmentsForValue(hund); digitalWrite(DIG1, COMMON_CATHODE ? LOW : HIGH); break;
    case 1: setSegmentsForValue(tens); digitalWrite(DIG2, COMMON_CATHODE ? LOW : HIGH); break;
    case 2: setSegmentsForValue(ones); digitalWrite(DIG3, COMMON_CATHODE ? LOW : HIGH); break;
  }
  curDigit = (curDigit + 1) % 3; // Next digit next time slice
}

/* ===== LEDs & buzzer ===== */
// Simple helpers to set R/Y/G state and buzzer state (active buzzer).
inline void setLEDs(bool r, bool y, bool g) {
  digitalWrite(LED_R, r?HIGH:LOW);
  digitalWrite(LED_Y, y?HIGH:LOW);
  digitalWrite(LED_G, g?HIGH:LOW);
}
inline void buzzerOn()  { digitalWrite(BUZZ, HIGH); }
inline void buzzerOff() { digitalWrite(BUZZ, LOW);  }

/* ===== Ultrasonic — interrupt-driven (non-blocking) =====
   ISR captures echo pulse start/end timestamps.
   latestDistanceCM() converts pulse width to cm and returns a 3-sample median for stability.
*/
volatile uint32_t echoStart = 0, echoEnd = 0; // Timestamps for ECHO pulse edges
volatile bool echoHigh = false, echoReady = false; // Flags for active pulse and availability

// ISR: fires on CHANGE (rising/falling). Records echoStart on rise, echoEnd on fall; sets echoReady when done.
void IR_echoChange() {
  if (digitalRead(ECHO_PIN)) { // Rising edge: pulse started
    echoHigh = true; echoReady = false; echoStart = micros();
  } else if (echoHigh) {       // Falling edge (only valid if we saw a rising first)
    echoEnd = micros(); echoHigh = false; echoReady = true;
  }
}

// Small ring buffer (3 samples) to median-filter distance readings (reduces spikes).
unsigned int distBuf[3] = {999,999,999};
uint8_t distIdx = 0;

// Returns last median distance in cm. If ISR posted a new pulse, compute duration->cm and update median.
unsigned int latestDistanceCM() {
  static unsigned int median = 999;
  if (echoReady) {
    noInterrupts();                 // Safely copy shared ISR vars
    uint32_t start = echoStart, end = echoEnd;
    echoReady = false;
    interrupts();

    // Handle micros() wrap-around: unsigned subtraction fixed by branch
    uint32_t dur = (end >= start) ? (end - start) : (0xFFFFFFFFu - start + end + 1u);

    // Convert µs to cm: HC-SR04 ~58µs per cm (round trip). Timeout >30ms => treat as "far" (999).
    distBuf[distIdx] = (dur > 30000UL) ? 999 : (unsigned int)(dur / 58UL);
    distIdx = (distIdx + 1) % 3;

    // Median of 3: sum - max - min
    unsigned int a = distBuf[0], b = distBuf[1], c = distBuf[2];
    unsigned int hi = max(a, max(b, c)), lo = min(a, min(b, c));
    median = a + b + c - hi - lo;
  }
  return median; // If no new reading, return last median
}

// Fires a single ultrasonic ping (10µs HIGH on TRIG). Called periodically in loop().
void triggerPing() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
}

/* ===== Servo sweep =====
   Sweeps head servo between SERVO_LEFT and SERVO_RIGHT at SERVO_STEP degrees per SERVO_SETTLE_MS.
*/
Servo head;
int servoAngle = SERVO_LEFT; // Current commanded angle
int servoDir   = +1;         // +1 => increasing angle (right), -1 => decreasing (left)

/* ===== DC Motor driver (non-blocking software PWM on EN) =====
   Motors use software PWM at 200Hz by toggling EN pin. IN1/IN2 set direction.
*/
const uint32_t PWM_PERIOD_US = 5000;       // 200 Hz PWM period
const uint8_t  SPEED_FAST    = 255;        // Full duty
const uint8_t  SPEED_SLOW    = 120;        // Reduced duty for cautious moves

// === "Option B" quick pivot: short in-place turn (one wheel fwd, other rev) before arc ===
const uint8_t  TURN_PIVOT_DUTY = 160;      // Duty during pivot burst (~63%)
const uint16_t TURN_PIVOT_MS   = 180;      // Pivot burst duration (ms)

/* ===== Anti-Stuck (close for too long) =====
   If stuck in RED for >2s, enter anti-stuck: reverse both wheels with asymmetry for 1.5s; rapid beep.
*/
const unsigned long ANTI_STUCK_TRIGGER_MS = 2000; // Time in continuous RED to trigger anti-stuck
const unsigned long ANTI_STUCK_ACTION_MS  = 1500; // Duration of anti-stuck maneuver
const unsigned long ANTI_STUCK_BEEP_MS    = 80;   // Beep toggle interval during anti-stuck

// Motor state container for software PWM + direction control.
struct DCMotor {
  uint8_t en, in1, in2;   // Pins: EN (PWM), IN1/IN2 (direction)
  int8_t  dir;            // +1 forward, -1 reverse
  bool    enabled;        // If false, EN driven LOW regardless
  uint8_t duty;           // 0..255 duty cycle
  bool    pwmHigh;        // Current EN state (HIGH during "on" portion)
  uint32_t nextToggleAt;  // Next micros() time to toggle EN
  uint32_t onTimeUs;      // Computed HIGH duration per period
  uint32_t offTimeUs;     // Computed LOW duration per period
};

// Instantiate two motors with defaults: forward, enabled, fast duty.
DCMotor m1{M1_EN, M1_IN1, M1_IN2, +1, true, SPEED_FAST, false, 0, 0, 0};
DCMotor m2{M2_EN, M2_IN1, M2_IN2, +1, true, SPEED_FAST, false, 0, 0, 0};

// Apply direction to IN1/IN2 pins for an H-bridge (IN1=HIGH/IN2=LOW => fwd; reversed => rev)
inline void dcApplyDir(const DCMotor& m) {
  if (m.dir >= 0) { digitalWrite(m.in1, HIGH); digitalWrite(m.in2, LOW);  }
  else            { digitalWrite(m.in1, LOW);  digitalWrite(m.in2, HIGH); }
}

// Compute on/off durations from duty for the fixed PWM period.
inline void dcComputeTimes(DCMotor& m) {
  if (m.duty >= 255)      { m.onTimeUs = PWM_PERIOD_US; m.offTimeUs = 0; }
  else if (m.duty == 0)   { m.onTimeUs = 0;             m.offTimeUs = PWM_PERIOD_US; }
  else {
    m.onTimeUs  = (uint32_t)((uint64_t)PWM_PERIOD_US * m.duty / 255u);
    m.offTimeUs = PWM_PERIOD_US - m.onTimeUs;
  }
}

// Initialize motor pins and schedule first PWM toggle.
void dcInit(DCMotor& m) {
  pinMode(m.en,  OUTPUT);
  pinMode(m.in1, OUTPUT);
  pinMode(m.in2, OUTPUT);
  digitalWrite(m.en, LOW);     // Start with EN low
  dcApplyDir(m);               // Apply initial direction
  dcComputeTimes(m);           // Precompute on/off times
  m.pwmHigh = false;           // Currently in "off" phase
  m.nextToggleAt = micros() + 500; // Slight delay before first toggle
}

// Set direction (normalized to +1/-1) and apply to pins immediately.
inline void dcSetDir(DCMotor& m, int8_t dir) { m.dir = (dir >= 0) ? +1 : -1; dcApplyDir(m); }
// Set duty and recompute timing.
inline void dcSetDuty(DCMotor& m, uint8_t duty) { m.duty = duty; dcComputeTimes(m); }
// Enable/disable motor; disabling forces EN LOW.
inline void dcEnable(DCMotor& m, bool on) { m.enabled = on; if (!on) digitalWrite(m.en, LOW); }

// Software PWM service: call frequently from loop() to maintain EN waveform.
void dcUpdate(DCMotor& m) {
  if (!m.enabled) { digitalWrite(m.en, LOW); return; }                 // Disabled => EN low
  if (m.onTimeUs == PWM_PERIOD_US) { digitalWrite(m.en, HIGH); return; } // 100% duty
  if (m.offTimeUs == PWM_PERIOD_US){ digitalWrite(m.en, LOW);  return; } // 0% duty

  const uint32_t now = micros();
  if ((int32_t)(now - m.nextToggleAt) < 0) return; // Not time to toggle

  if (m.pwmHigh) { // Currently ON: switch OFF
    digitalWrite(m.en, LOW);
    m.pwmHigh = false;
    m.nextToggleAt = now + m.offTimeUs;
  } else {         // Currently OFF: switch ON
    digitalWrite(m.en, HIGH);
    m.pwmHigh = true;
    m.nextToggleAt = now + m.onTimeUs;
  }
}

/* ===== Control tiers for DC speed =====
   Zone model: GREEN (fast forward), YELLOW (slow forward), RED (avoidance).
   RED behavior:
     1) quick pivot (opposite wheel directions) for TURN_PIVOT_MS
     2) then arc turn: one side OFF, other SLOW forward
*/
enum Zone { Z_GREEN, Z_YELLOW, Z_RED };
Zone lastZone = Z_GREEN; // Tracks prior zone (note: code also uses a separate 'prev' inside loop)
int stoppedMotor = -1;   // Index of the side to stop during arc (0=m1, 1=m2), -1=none

// Red "Option B" pivot timing control (0 => not pivoting)
unsigned long pivotUntil = 0;

// Anti-stuck state: after prolonged RED, reverse+turn with rapid beeps.
unsigned long redSince = 0;         // When RED started (0 if not in RED)
bool          antiStuck = false;    // Anti-stuck active flag
unsigned long antiStuckUntil = 0;   // When to end anti-stuck
int           antiStuckDir = 0;     // 0 => bias left, 1 => bias right during reverse
unsigned long nextBeepToggleAt = 0; // Rapid beep scheduler
bool          beepOn = false;       // Current buzzer state in anti-stuck

/* ===== Setup ===== */
void setup() {
  // --- IO directions ---
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);      // Must be level-shifted to 3.3V on DUE
  for (uint8_t i=0;i<7;i++) pinMode(SEG_PINS[i], OUTPUT);
  for (uint8_t i=0;i<3;i++) pinMode(DIG_PINS[i], OUTPUT);
  pinMode(LED_R, OUTPUT); pinMode(LED_Y, OUTPUT); pinMode(LED_G, OUTPUT);
  pinMode(BUZZ, OUTPUT);

  // --- DC motors initial state ---
  dcInit(m1); dcInit(m2);
  dcSetDir(m1, +1);  dcSetDir(m2, +1);        // Start forward (flip if wiring reversed)
  dcSetDuty(m1, SPEED_FAST); dcSetDuty(m2, SPEED_FAST);
  dcEnable(m1, true); dcEnable(m2, true);

  // --- Random seed for unbiased left/right choices (uses analog noise + current time) ---
  randomSeed( (uint32_t) (analogRead(A0) + micros()) );

  // --- Display & indicators initial state ---
  allDigitsOff();
  // Servo attached & positioned to left limit
  head.attach(SERVO_PIN);
  head.write(servoAngle);
  // Start with GREEN indicator; buzzer off
  setLEDs(false,false,true);
  buzzerOff();

  // --- Ultrasonic echo edge ISR (captures pulse widths without blocking) ---
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), IR_echoChange, CHANGE);

  // --- Kick off first ping after brief delay; schedule display refresh ---
  delay(50);
  triggerPing();
  nextDisplayAt = millis() + 10;        // Update displayValue at 100Hz
  nextDigitAt = micros() + DIGIT_PERIOD_US; // First multiplex slice
}

/* ===== Loop ===== */
void loop() {
  // 1) Keep display alive (multiplex one digit at a time)
  refreshDisplaySoft();

  // 2) Sweep servo steadily (even during anti-stuck)
  static unsigned long nextMoveAt = 0;
  if ((long)(millis() - nextMoveAt) >= 0) {
    nextMoveAt = millis() + SERVO_SETTLE_MS;
    servoAngle += servoDir * SERVO_STEP;
    if (servoAngle >= SERVO_RIGHT) { servoAngle = SERVO_RIGHT; servoDir = -1; } // Hit right limit: go left
    if (servoAngle <= SERVO_LEFT)  { servoAngle = SERVO_LEFT;  servoDir = +1; } // Hit left limit: go right
    head.write(servoAngle);
  }

  // 3) Periodically trigger ultrasonic pings (~25Hz). During anti-stuck, ignore new reads (no pings).
  static unsigned long nextPingAt = 0;
  if (!antiStuck) {
    if ((long)(millis() - nextPingAt) >= 0) { nextPingAt = millis() + 40; triggerPing(); }
  }

  // 4) Read latest measured distance (median of last 3). If no new echo, returns previous value.
  unsigned int cm = latestDistanceCM();

  // 5) Determine zone; anti-stuck forces RED handling regardless of current cm.
  Zone z;
  if (antiStuck) {
    z = Z_RED;  // Lock behavior to RED during anti-stuck
  } else {
    if (cm > (unsigned)DIST_WARN_CM)       z = Z_GREEN;
    else if (cm > (unsigned)DIST_CLOSE_CM) z = Z_YELLOW;
    else                                   z = Z_RED;
  }

  // 6) Track time spent in RED to trigger anti-stuck if stuck too long.
  if (!antiStuck) {
    if (z == Z_RED) {
      if (redSince == 0) redSince = millis(); // Just entered RED
      else if ((long)(millis() - redSince) >= (long)ANTI_STUCK_TRIGGER_MS) {
        // --- Enter Anti-Stuck: reverse+turn burst with rapid beep ---
        antiStuck = true;
        antiStuckUntil = millis() + ANTI_STUCK_ACTION_MS;
        antiStuckDir = random(0, 2);           // Randomize turn bias (0=left,1=right)
        nextBeepToggleAt = millis(); beepOn = false;

        pivotUntil = 0;                         // Cancel any ongoing pivot
        dcSetDir(m1, -1); dcSetDir(m2, -1);    // Both reverse
        if (antiStuckDir == 0) {
          // Bias left: left faster, right slower (reverse)
          dcEnable(m1, true);  dcSetDuty(m1, 200);
          dcEnable(m2, true);  dcSetDuty(m2, 120);
        } else {
          // Bias right: right faster, left slower (reverse)
          dcEnable(m2, true);  dcSetDuty(m2, 200);
          dcEnable(m1, true);  dcSetDuty(m1, 120);
        }
      }
    } else {
      redSince = 0; // Left RED; reset timer
    }
  }

  // 7) Zone transition handling (not during anti-stuck). One-time actions per entry into a zone.
  static Zone prev = Z_GREEN; // Tracks last applied zone reaction
  if (!antiStuck && z != prev) {
    switch (z) {
      case Z_GREEN:
        // Fast forward, both enabled, cancel pivots
        dcSetDir(m1, +1); dcSetDir(m2, +1);
        dcSetDuty(m1, SPEED_FAST); dcSetDuty(m2, SPEED_FAST);
        dcEnable(m1, true); dcEnable(m2, true);
        stoppedMotor = -1;
        pivotUntil = 0;
        break;

      case Z_YELLOW:
        // Slow forward, both enabled, cancel pivots
        dcSetDir(m1, +1); dcSetDir(m2, +1);
        dcSetDuty(m1, SPEED_SLOW); dcSetDuty(m2, SPEED_SLOW);
        dcEnable(m1, true); dcEnable(m2, true);
        stoppedMotor = -1;
        pivotUntil = 0;
        break;

      case Z_RED:
        // Immediate short pivot burst: one motor rev, other fwd, to reorient quickly
        stoppedMotor = random(0, 2); // Remember which side will later be stopped in arc
        if (stoppedMotor == 0) {
          // Pivot left
          dcSetDir(m1, -1); dcEnable(m1, true); dcSetDuty(m1, TURN_PIVOT_DUTY); // Left backward
          dcSetDir(m2, +1); dcEnable(m2, true); dcSetDuty(m2, TURN_PIVOT_DUTY); // Right forward
        } else {
          // Pivot right
          dcSetDir(m2, -1); dcEnable(m2, true); dcSetDuty(m2, TURN_PIVOT_DUTY); // Right backward
          dcSetDir(m1, +1); dcEnable(m1, true); dcSetDuty(m1, TURN_PIVOT_DUTY); // Left forward
        }
        pivotUntil = millis() + TURN_PIVOT_MS; // End of pivot window
        break;
    }
    prev = z; // Record applied zone
  }

  // 8) After pivot window expires (still RED), transition to arc: stop one side, other slow forward.
  if (!antiStuck && z == Z_RED && pivotUntil != 0 && (long)(millis() - pivotUntil) >= 0) {
    dcSetDir(m1, +1); dcSetDir(m2, +1); // Arc uses forward direction on the moving side
    if (stoppedMotor == 0) {
      // Stop left, slow right => turn left
      dcEnable(m1, false);
      dcEnable(m2, true);  dcSetDuty(m2, SPEED_SLOW);
    } else {
      // Stop right, slow left => turn right
      dcEnable(m2, false);
      dcEnable(m1, true);  dcSetDuty(m1, SPEED_SLOW);
    }
    pivotUntil = 0; // Done with pivot phase
  }

  // 9) Anti-stuck maintenance: rapid beeps and timed exit.
  if (antiStuck) {
    // Toggle buzzer at ANTI_STUCK_BEEP_MS intervals
    if ((long)(millis() - nextBeepToggleAt) >= 0) {
      nextBeepToggleAt = millis() + ANTI_STUCK_BEEP_MS;
      beepOn = !beepOn;
      if (beepOn) buzzerOn(); else buzzerOff();
    }

    // Exit anti-stuck when time elapses; face forward and force zone re-evaluation.
    if ((long)(millis() - antiStuckUntil) >= 0) {
      antiStuck = false;
      redSince = 0;
      beepOn = false; buzzerOff();

      dcSetDir(m1, +1); dcSetDir(m2, +1); // Prepare to resume normal logic forward
      // Force zone-change switch to run next iteration regardless of current z
      prev = (Zone)255; // Invalid marker so (z != prev) is guaranteed true next loop
    }
  }

  // 10) LEDs, buzzer (outside anti-stuck), and display value update request.
  if (!antiStuck) {
    if (z == Z_GREEN) {
      setLEDs(false,false,true);
      pendingDisplayValue = (cm <= MAX_DISPLAY_CM) ? (int)cm : MAX_DISPLAY_CM;
      buzzerOff();
    } else if (z == Z_YELLOW) {
      setLEDs(false,true,false);
      pendingDisplayValue = (cm <= MAX_DISPLAY_CM) ? (int)cm : MAX_DISPLAY_CM;
      buzzerOff();
    } else {
      setLEDs(true,false,false);
      pendingDisplayValue = (cm <= MAX_DISPLAY_CM) ? (int)cm : MAX_DISPLAY_CM;
      buzzerOn();
    }
  } else {
    // In anti-stuck: force RED LED; buzzer handled by rapid toggle above; still show distance.
    setLEDs(true,false,false);
    pendingDisplayValue = (cm <= MAX_DISPLAY_CM) ? (int)cm : MAX_DISPLAY_CM;
  }

  // 11) Apply display update at 100Hz (decoupled from sensor read rate).
  if ((long)(millis() - nextDisplayAt) >= 0) {
    nextDisplayAt = millis() + 10;
    displayValue = pendingDisplayValue;
  }

  // 12) Drive software PWM for both motors (must be called frequently).
  dcUpdate(m1);
  dcUpdate(m2);
}

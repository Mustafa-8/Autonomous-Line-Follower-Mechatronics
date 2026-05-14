/*
 * 5-sensor line follower with PD control.
 *
 * Each loop: read sensors → compute weighted-average error →
 * classify turn sharpness → pick base speed → apply PID correction →
 * differential drive (left = base + correction, right = base - correction).
 *
 * Hardware: Arduino + TB6612FNG motor driver + 5 digital IR sensors.
 */

// All pin numbers use uint8_t (unsigned 8-bit int, range 0-255) to save RAM.

// --- MOTOR PINS (TB6612FNG) ---
const uint8_t STBY = 6;   // Standby — HIGH wakes up the motor driver
const uint8_t PWMA = 3;   // Right motor speed control (PWM)
const uint8_t AIN1 = 7;   // Direction control for motor A (right)
const uint8_t AIN2 = 8;   // Direction control for motor A (right)
const uint8_t PWMB = 5;   // Left motor speed control (PWM)
const uint8_t BIN1 = 4;   // Direction control for motor B (left)
const uint8_t BIN2 = 2;   // Direction control for motor B (left)

// --- SENSOR PINS (Far Left -> Far Right) ---
const uint8_t SENSOR_PINS[5] = {A4, A3, A2, A1, A0};
const int8_t  WEIGHTS[5]     = {-4, -2, 0, 2, 4};   // -4: far left | 0: center | +4: far right
                                                    // Negative = line is left, positive = line is right

// --- SPEED PROFILES (FULL SPEED) ---
// The +9 difference between right and left compensates for the left motor being slightly stronger.
const int RIGHT_STRAIGHT = 220, LEFT_STRAIGHT = 211;
const int RIGHT_CURVE    = 160, LEFT_CURVE    = 151;
const int RIGHT_SHARP    = 90,  LEFT_SHARP    = 81;
const int RIGHT_LOST     = 70,  LEFT_LOST     = 61;
const int MAX_SPEED      = 255;

// --- PID GAINS ---
const int Kp = 12;    // Proportional — how hard to react to current error.
                      // Higher Kp = sharper reactions but more wobble.
const int Kd = 14;    // Derivative — how hard to react to CHANGE in error.
                      // Higher Kd = smoother and more stable, but too high becomes sluggish.

int    lastError    = 0;    // Error from previous loop (needed for Kd calculation)
int8_t lastTurnDir  = 0;    // Last non-zero turn direction (-1 left, +1 right). Used when line is lost.

void setup() {
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  for (uint8_t i = 0; i < 5; i++) pinMode(SENSOR_PINS[i], INPUT);

  // Make sure motors are stopped before we wake the driver
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);

  digitalWrite(STBY, HIGH);   // Wake up the motor driver
  delay(2000);                // Start delay so you have time to place the bot on the track
}

void loop() {
  // --- READ ALL 5 SENSORS + WEIGHTED AVERAGE ---
  // For every sensor that sees the line, add its weight to sum and count it.
  // Then error = sum / count gives the weighted-average position of the line.
  uint8_t s[5];
  uint8_t count = 0;
  int     sum   = 0;

  for (uint8_t i = 0; i < 5; i++) {
    s[i] = digitalRead(SENSOR_PINS[i]);   // 0 = white, 1 = black line
    if (s[i]) {
      sum   += WEIGHTS[i];
      count += 1;
    }
  }

  // --- ERROR + TURN TYPE ---
  int     error;
  bool    lineDetected = (count > 0);
  uint8_t turnType;

  if (lineDetected) {
    error = sum / count;

    // Classify turn based on how far off-center the line is (sign doesn't matter here).
    uint8_t absErr = (error < 0) ? -error : error;
    if      (absErr == 0) turnType = 0;   // straight
    else if (absErr <= 2) turnType = 1;   // curve
    else                  turnType = 2;   // sharp turn

    // Remember which side the line was on. IMPORTANT: only update on non-zero error
    // so that if the line vanishes right at center, we still have a recovery direction.
    if      (error < 0) lastTurnDir = -1;
    else if (error > 0) lastTurnDir =  1;
    // (if error == 0, keep previous lastTurnDir)
  } else {
    // Line lost: fake the error to force a turn in the last known direction.
    error    = (lastTurnDir < 0) ? -4 : (lastTurnDir > 0) ? 4 : lastError;
    turnType = 3;   // lost
  }

  // --- BASE SPEEDS ---
  int baseRight, baseLeft;
  switch (turnType) {
    case 0: baseRight = RIGHT_STRAIGHT; baseLeft = LEFT_STRAIGHT; break;
    case 1: baseRight = RIGHT_CURVE;    baseLeft = LEFT_CURVE;    break;
    case 2: baseRight = RIGHT_SHARP;    baseLeft = LEFT_SHARP;    break;
    default:baseRight = RIGHT_LOST;     baseLeft = LEFT_LOST;     break;   // turnType 3 (lost)
  }

  // --- PID ---
  int derivative = error - lastError;
  int correction = (Kp * error) + (Kd * derivative);

  if (lineDetected) lastError = error;   // Only save real errors (not the faked "lost" ones)

  // --- APPLY CORRECTION TO MOTORS ---
  // Differential steering: one wheel speeds up, the other slows down (or reverses on sharp turns).
  // Positive correction → turn right.  Negative correction → turn left.
  int leftSpeed  = constrain(baseLeft  + correction, -MAX_SPEED, MAX_SPEED);
  int rightSpeed = constrain(baseRight - correction, -MAX_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);
}

// --- MOTOR CONTROL ---
// Negative speed = reverse direction. AIN1/AIN2 set direction, PWMA sets magnitude.
// Left motor (channel B) pins are inverted vs right because of how channel B is physically wired.
void setMotors(int leftSpeed, int rightSpeed) {
  // RIGHT motor (channel A)
  digitalWrite(AIN1, rightSpeed >= 0 ? HIGH : LOW);
  digitalWrite(AIN2, rightSpeed >= 0 ? LOW  : HIGH);
  analogWrite(PWMA, abs(rightSpeed));

  // LEFT motor (channel B) — pins inverted on purpose
  digitalWrite(BIN1, leftSpeed >= 0 ? LOW  : HIGH);
  digitalWrite(BIN2, leftSpeed >= 0 ? HIGH : LOW);
  analogWrite(PWMB, abs(leftSpeed));
}

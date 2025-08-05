//PID Line Following Bot using Arduino Uno

// Motor Driver Pins (L298N)
#define ENA 5    // Left Motor Speed (PWM)
#define ENB 10   // Right Motor Speed (PWM)
#define IN1 7    // Left Motor Direction 1
#define IN2 6    // Left Motor Direction 2
#define IN3 4    // Right Motor Direction 1
#define IN4 3    // Right Motor Direction 2

// Waveshare 5 IR sensor array analog pins
#define SENSOR1 A4 // Leftmost
#define SENSOR2 A3
#define SENSOR3 A2
#define SENSOR4 A1
#define SENSOR5 A0 // Rightmost

// ======= Tunables (Calibrate these for your surface) =======
int THRESH_LINE = 400;       // (1023 - analogRead) above this => on black line
int THRESH_90   = 850;       // high reflect (inverted) for strong line at edges
int SUM_90      = 1800;      // total inverted sum gate for junctions

// 90° turn behavior
const int HOLD_MS_90      = 30;   // condition must hold this long to confirm junction
const int BRAKE_MS_BEFORE = 40;   // quick brake before pivot
const int TURN_PWM        = 120;  // pivot PWM (raise for faster spins if traction allows)
const int TURN_TIMEOUT_MS = 450;  // safety timeout during pivot
const int RECENTER_MS     = 80;   // creep forward after pivot to stabilize
const int SEARCH_COOLDOWN = 150;  // avoid instantly re-entering search after reacquiring

// PID Constants
//Adjust these parameters based on your project
float Kp = 0.35;
float Ki = 0.001;
float Kd = 3.5;

// PID Variables and speeds
int lastError = 0;
float integral = 0;
int baseSpeed = 75;     // min sustain speed
int maxSpeed  = 140;    // cap speed

// Smoothing filter
#define FILTER_SIZE 5
int positionHistory[FILTER_SIZE] = {2000, 2000, 2000, 2000, 2000};
int filterIndex = 0;

// State for line-loss recovery
static unsigned long lineLostTime = 0;
static bool searching = false;
static unsigned long searchCooldownUntil = 0;

// ---------- Forward Declarations ----------
void driveMotors(int left, int right);
void stopMotors();
void printStatus(int sensors[5], int position, int error, int leftSpeed, int rightSpeed);
int  getLinePosition(int sensors[5]);
int  getSmoothedPosition(int newPosition);
void readSensors(int s[5]);
bool isJunctionInv(int inv[5]);
bool confirmJunction();
bool pivotUntilCenter(bool turnLeft);

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(SENSOR1, INPUT); pinMode(SENSOR2, INPUT);
  pinMode(SENSOR3, INPUT); pinMode(SENSOR4, INPUT);
  pinMode(SENSOR5, INPUT);

  stopMotors();
  Serial.println("Fast PID Line Follower + Robust 90° Turns Ready");
  delay(1200);
}

// ------------------- Loop -------------------
void loop() {
  int sensors[5];
  readSensors(sensors);

  // Invert once for convenience
  int inv[5];
  int sumInv = 0;
  for (int i = 0; i < 5; i++) {
    inv[i] = 1023 - sensors[i];
    sumInv += inv[i];
  }

  int rawPosition = getLinePosition(sensors);
  int position = getSmoothedPosition(rawPosition);

  // Detect if any sensor sees the line
  bool onLine = false;
  for (int i = 0; i < 5; i++) {
    if (inv[i] > THRESH_LINE) {
      onLine = true;
      break;
    }
  }

  // Short cooldown after search to avoid immediate retrigger
  if (millis() < searchCooldownUntil) onLine = true;

  int error = 2000 - position;  // If steering is reversed, flip sign to (position - 2000)
  int absError = abs(error);

  // ======= Fast Line Loss Recovery ========
  if (!onLine) {
    if (!searching) {
      searching = true;
      lineLostTime = millis();
      integral = 0;
      lastError = 0;
    }

    unsigned long elapsed = millis() - lineLostTime;

    if (elapsed < 150) {
      driveMotors(-100, -100);  // Quick reverse
    } else if (elapsed < 350) {
      driveMotors(120, -120);   // Fast spin right
    } else {
      lineLostTime = millis();  // restart phases
    }

    delay(30);
    return;
  } else {
    if (searching) {
      searching = false;
      searchCooldownUntil = millis() + SEARCH_COOLDOWN;
    }
  }

  // ======= Robust & Fast 90° Detection + Turn =======
  // Gate: high sum (junction), edge pair high, center low; debounce via confirmJunction()
  bool leftPair  = (inv[0] > THRESH_90) && (inv[1] > THRESH_90);
  bool rightPair = (inv[4] > THRESH_90) && (inv[3] > THRESH_90);
  bool centerLow = inv[2] < THRESH_LINE;
  bool likelyJunction = (sumInv > SUM_90) && centerLow && (leftPair || rightPair);

  if (likelyJunction && confirmJunction()) {
    // Decide direction by which side is stronger
    int leftPower = inv[0] + inv[1];
    int rightPower = inv[4] + inv[3];
    bool turnLeft = leftPower > rightPower;

    // Short brake
    driveMotors(0, 0);
    delay(BRAKE_MS_BEFORE);

    // Adaptive pivot: rotate until center sees line (or timeout)
    if (pivotUntilCenter(turnLeft)) {
      // Recenter creep forward briefly
      driveMotors(90, 90);
      delay(RECENTER_MS);
    }
    return; // skip PID this loop
  }

  // ======= PID Line Following =======
  if (absError < 40) {
    integral = 0;
    error = 0;
  }   // deadband

  int P = error;
  int D = error - lastError;
  integral += error;
  lastError = error;

  if (integral > 800) integral = 800;
  if (integral < -800) integral = -800;

  float PID = (Kp * P) + (Ki * integral) + (Kd * D);

  // Adaptive base speed by error (gives agility but preserves top speed on big errors)
  float speedMultiplier = 1.2;
  if (absError >= 1200) {
    speedMultiplier = 0.9; // if you want more slowing at very large errors, lower this (e.g., 0.6)
  } else if (absError >= 600) {
    speedMultiplier = 0.7 + 0.2 * (1.0 - float(absError - 600) / 600.0); // ~0.9 down to ~0.7
  }

  int adjustedBaseSpeed = max(65, int(baseSpeed * speedMultiplier));
  if (absError > 1000) adjustedBaseSpeed = min(maxSpeed, adjustedBaseSpeed + 30);

  int leftSpeed  = adjustedBaseSpeed - PID;
  int rightSpeed = adjustedBaseSpeed + PID;

  leftSpeed  = constrain(leftSpeed,  65, maxSpeed);
  rightSpeed = constrain(rightSpeed, 65, maxSpeed);

  driveMotors(leftSpeed, rightSpeed);
  printStatus(sensors, position, error, leftSpeed, rightSpeed);

  delay(20);
}

// ------------------- Utility -------------------

void readSensors(int s[5]) {
  s[0] = analogRead(SENSOR1);
  s[1] = analogRead(SENSOR2);
  s[2] = analogRead(SENSOR3);
  s[3] = analogRead(SENSOR4);
  s[4] = analogRead(SENSOR5);
}

int getLinePosition(int sensors[5]) {
  long weightedSum = 0;
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    int value = 1023 - sensors[i];    // invert if dark line gives lower ADC
    weightedSum += (long)value * (i * 1000); // positions: 0,1000,2000,3000,4000
    sum += value;
  }
  if (sum == 0) return 2000;           // neutral if nothing seen
  return weightedSum / sum;            // 0..4000 (center ~2000)
}

int getSmoothedPosition(int newPosition) {
  positionHistory[filterIndex] = newPosition;
  filterIndex = (filterIndex + 1) % FILTER_SIZE;
  long total = 0;
  for (int i = 0; i < FILTER_SIZE; i++) total += positionHistory[i];
  return total / FILTER_SIZE;
}

bool isJunctionInv(int inv[5]) {
  int sum = inv[0] + inv[1] + inv[2] + inv[3] + inv[4];
  bool leftPair  = inv[0] > THRESH_90 && inv[1] > THRESH_90;
  bool rightPair = inv[4] > THRESH_90 && inv[3] > THRESH_90;
  bool centerLow = inv[2] < THRESH_LINE;
  return (sum > SUM_90) && centerLow && (leftPair || rightPair);
}

bool confirmJunction() {
  // Hold condition stable for HOLD_MS_90
  unsigned long t0 = millis();
  int inv[5];
  while (millis() - t0 < (unsigned long)HOLD_MS_90) {
    int s[5]; 
    readSensors(s);
    for (int i = 0; i < 5; i++) inv[i] = 1023 - s[i];
    if (!isJunctionInv(inv)) return false;
    delay(5);
  }
  return true;
}

bool pivotUntilCenter(bool turnLeft) {
  unsigned long t0 = millis();
  int center = 0;
  while (millis() - t0 < (unsigned long)TURN_TIMEOUT_MS) {
    // Pivot in place
    int l = turnLeft ? -TURN_PWM : TURN_PWM;
    int r = turnLeft ? TURN_PWM : -TURN_PWM;
    driveMotors(l, r);

    // Check center sensor
    int s2 = analogRead(SENSOR3);
    center = 1023 - s2;
    if (center > THRESH_LINE) {
      return true; // line reacquired
    }
    delay(5);
  }
  // Timeout: stop briefly to avoid runaway
  driveMotors(0, 0);
  return false;
}

void driveMotors(int left, int right) {
  // Left motor
  if (left >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, left);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    analogWrite(ENA, -left);
  }
  // Right motor
  if (right >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENB, right);
  } else {
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    analogWrite(ENB, -right);
  }
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void printStatus(int sensors[5], int position, int error, int leftSpeed, int rightSpeed) {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("IR:");
    for (int i = 0; i < 5; i++) {
      Serial.print(1023 - sensors[i]);
      Serial.print(' ');
    }
    Serial.print("| Pos:"); Serial.print(position);
    Serial.print(" Err:"); Serial.print(error);
    Serial.print(" L:"); Serial.print(leftSpeed);
    Serial.print(" R:"); Serial.println(rightSpeed);
    lastPrint = millis();
  }
}

/*
  GESTURE-CONTROLLED LIGHT INTENSITY USING DUAL ULTRASONIC SENSORS
  ------------------------------------------------------------------
  Logic:
  1. Show palm anywhere in front of both sensors.
  2. Hold it still for 2 seconds -> system locks that position as baseline (zero point).
  3. Move hand LEFT -> RIGHT  : intensity increases
     Move hand RIGHT -> LEFT : intensity decreases
  4. Full 0-255 intensity swing happens over 30cm of lateral movement from the locked point.
  5. Remove hand from range -> system resets and waits for a new palm + new calibration.

  Wiring:
  Left HC-SR04  : TRIG -> D9   ECHO -> D10   VCC -> 5V   GND -> GND
  Right HC-SR04 : TRIG -> D11  ECHO -> D12   VCC -> 5V   GND -> GND
  LED (PWM)     : Anode -> D6 (through 220ohm resistor) -> Cathode -> GND
  Buzzer        : +    -> D5   -    -> GND
  LCD I2C       : SDA -> A4    SCL -> A5     VCC -> 5V   GND -> GND
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // change to 0x3F if your LCD doesn't show text at 0x27

// ---------- Pin definitions ----------
const int TRIG_LEFT  = 9;
const int ECHO_LEFT  = 10;
const int TRIG_RIGHT = 11;
const int ECHO_RIGHT = 12;
const int LED_PIN    = 6;
const int BUZZER_PIN = 5;

// ---------- Tunable constants ----------
const float MIN_RANGE_CM   = 5.0;   // closest valid reading
const float MAX_RANGE_CM   = 30.0;  // beyond this = "no hand" / idle
const float STABLE_TOL_CM  = 3.5;   // allowed jitter during calibration hold
const unsigned long HOLD_TIME_MS = 500; //0.5 sec stillness required to lock
const float MOVEMENT_RANGE_CM = 15.0;    // 15cm movement = full 0-255 swing
const int FILTER_SAMPLES = 8;            // moving average window size

// ---------- State machine ----------
enum State { IDLE, CALIBRATING, LOCKED, TRACKING };
State currentState = IDLE;

// ---------- Rolling buffers for filtering ----------
float leftBuffer[FILTER_SAMPLES];
float rightBuffer[FILTER_SAMPLES];
int bufferIndex = 0;
bool bufferFilled = false;

// ---------- Calibration tracking ----------
unsigned long stableStartTime = 0;
float calibAnchorLeft = 0;
float calibAnchorRight = 0;

// ---------- Locked baseline ----------
float baselineDiff = 0;

// ---------- Current output ----------
int currentIntensity = 0;

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Show palm to");
  lcd.setCursor(0, 1);
  lcd.print("begin...");

  // initialize buffers
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    leftBuffer[i] = MAX_RANGE_CM + 10;
    rightBuffer[i] = MAX_RANGE_CM + 10;
  }
}

void loop() {
  float rawLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
  float rawRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

  updateBuffer(leftBuffer, rawLeft);
  updateBuffer(rightBuffer, rawRight);
  bufferIndex = (bufferIndex + 1) % FILTER_SAMPLES; // advance once per cycle, shared by both buffers

  float left = getAverage(leftBuffer);
  float right = getAverage(rightBuffer);

  bool handPresent = (left >= MIN_RANGE_CM && left <= MAX_RANGE_CM) &&
                      (right >= MIN_RANGE_CM && right <= MAX_RANGE_CM);

  switch (currentState) {

    case IDLE:
      analogWrite(LED_PIN, 0);
      if (handPresent) {
        currentState = CALIBRATING;
        stableStartTime = millis();
        calibAnchorLeft = left;
        calibAnchorRight = right;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Hold steady...");
      }
      break;

    case CALIBRATING:
      if (!handPresent) {
        // hand removed mid-calibration, go back to idle
        currentState = IDLE;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Show palm to");
        lcd.setCursor(0, 1);
        lcd.print("begin...");
        break;
      }

      // check if still within tolerance of the anchor
      if (abs(left - calibAnchorLeft) > STABLE_TOL_CM ||
          abs(right - calibAnchorRight) > STABLE_TOL_CM) {
        // moved too much, restart the 2 sec timer from current position
        stableStartTime = millis();
        calibAnchorLeft = left;
        calibAnchorRight = right;
      }

      // show countdown progress
      {
        unsigned long elapsed = millis() - stableStartTime;
        lcd.setCursor(0, 1);
        lcd.print("Lock in: ");
        lcd.print(elapsed / 1000.0, 1);
        lcd.print("s ");

        if (elapsed >= HOLD_TIME_MS) {
          // LOCK IN
          baselineDiff = left - right;
          currentState = LOCKED;
        }
      }
      break;

    case LOCKED:
      tone(BUZZER_PIN, 1000, 150); // short beep to confirm lock
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Locked! Move");
      lcd.setCursor(0, 1);
      lcd.print("hand to adjust");
      delay(400); // let the message show briefly
      currentState = TRACKING;
      break;

    case TRACKING:
      if (!handPresent) {
        // hand left range, reset everything
        currentState = IDLE;
        analogWrite(LED_PIN, 0);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Show palm to");
        lcd.setCursor(0, 1);
        lcd.print("begin...");
        break;
      }

      {
        float currentDiff = left - right;
        float deltaDiff = currentDiff - baselineDiff;

        // map deltaDiff (-MOVEMENT_RANGE_CM to +MOVEMENT_RANGE_CM) to (0 to 255)
        long mapped = map((long)(deltaDiff * 10), 
                           (long)(-MOVEMENT_RANGE_CM * 10), 
                           (long)(MOVEMENT_RANGE_CM * 10), 
                           0, 255);
        currentIntensity = constrain(mapped, 0, 255);

        analogWrite(LED_PIN, currentIntensity);

        lcd.setCursor(0, 0);
        lcd.print("L:");
        lcd.print((int)left);
        lcd.print(" R:");
        lcd.print((int)right);
        lcd.print("    ");

        lcd.setCursor(0, 1);
        lcd.print("Intensity:");
        lcd.print(currentIntensity);
        lcd.print("   ");
      }
      break;
  }

  Serial.print("State: ");
  Serial.print(currentState);
  Serial.print(" | Left: ");
  Serial.print(left);
  Serial.print(" | Right: ");
  Serial.print(right);
  Serial.print(" | Intensity: ");
  Serial.println(currentIntensity);

  delay(30); // ~30ms sampling interval
}

// ---------- Helper: get raw distance from one HC-SR04 ----------
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout (~5m max range)
  if (duration == 0) {
    return MAX_RANGE_CM + 10; // treat timeout as "nothing detected"
  }
  float distance = (duration * 0.0343) / 2.0; // speed of sound = 0.0343 cm/us
  return distance;
}

// ---------- Helper: update rolling buffer with new reading ----------
void updateBuffer(float *buffer, float newValue) {
  buffer[bufferIndex] = newValue;
}

// ---------- Helper: get moving average of a buffer ----------
float getAverage(float *buffer) {
  float sum = 0;
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    sum += buffer[i];
  }
  return sum / FILTER_SAMPLES;
}

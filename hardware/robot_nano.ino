// robot_nano.ino
// Arduino Nano: Ultrasonic + TB6612FNG parser for tank commands "T<left>,<right>\n"

#include <NewPing.h> // optional but convenient. If not available, use manual pulse.
#define USE_NEWPING 0

// Motor A pins
const int PWMA = 5;
const int AIN1 = 7;
const int AIN2 = 6;

// Motor B pins
// Motor B pins
// NOTE: must be a PWM-capable digital pin so analogWrite() produces PWM.
// A5 on many Arduino boards is not a PWM pin; using a PWM pin (e.g., D10) prevents
// incorrect motor behaviour. Update wiring if you change this.
const int PWMB = 10;  // PWM-capable digital pin (was A5)
const int BIN1 = A3;
const int BIN2 = A4;

// Ultrasonic pins
const int TRIG_PIN = 8;
const int ECHO_PIN = 9;

unsigned long lastUsMillis = 0;
const unsigned long US_INTERVAL = 150; // ms

void setup() {
  Serial.begin(115200);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // default stop
  stopMotors();
}

void loop() {
  // Read serial commands non-blocking
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) parseCommand(line);
  }

  // Periodically send ultrasonic reading
  unsigned long t = millis();
  if (t - lastUsMillis >= US_INTERVAL) {
    lastUsMillis = t;
    float d = readUltrasonic();
    // send as meters with 3 decimal places
    char buf[32];
    dtostrf(d, 0, 3, buf);
    Serial.print("US ");
    Serial.print(buf);
    Serial.print("\n");
  }
}

// ---------- Ultrasonic read (meters) ----------
float readUltrasonic() {
#if USE_NEWPING
  // if using NewPing library (not used by default)
  // not implemented here by default
  return -1.0;
#else
  // classic ping
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  if (duration == 0) return -1.0;
  float distance_cm = (duration / 2.0) / 29.1;
  return distance_cm / 100.0;
#endif
}

// ---------- Command parser ----------
void parseCommand(String s) {
  // expected: T<left>,<right>
  if (s.length() == 0) return;
  if (s.charAt(0) == 'T') {
    int comma = s.indexOf(',');
    if (comma <= 1) return;
    String sl = s.substring(1, comma);
    String sr = s.substring(comma + 1);
    sl.trim(); sr.trim();
    int left = sl.toInt();
    int right = sr.toInt();
    left = max(-255, min(255, left));
    right = max(-255, min(255, right));
    setTank(left, right);
  } else if (s == "S" || s == "STOP") {
    stopMotors();
  } else if (s.length() > 0) {
    // unknown, ignore
  }
}

// ---------- Motor helpers ----------
void setMotor(int pwmpin, int in1, int in2, int value) {
  // value: -255..255
  if (value == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmpin, 0);
  } else if (value > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmpin, value);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmpin, -value);
  }
}

void setTank(int left, int right) {
  setMotor(PWMA, AIN1, AIN2, left);
  setMotor(PWMB, BIN1, BIN2, right);
}

void stopMotors() {
  setMotor(PWMA, AIN1, AIN2, 0);
  setMotor(PWMB, BIN1, BIN2, 0);
}

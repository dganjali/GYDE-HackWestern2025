// Minimal Arduino Nano controller for MVP pipeline
// Receives messages like: L:0.123;A:-0.456\n
#include <Arduino.h>

// Motor pins (example, change to match your wiring)
#define LEFT_PWM 5
#define RIGHT_PWM 6
#define LEFT_DIR 7
#define RIGHT_DIR 8

// Safety pins
#define ESTOP_PIN 4        // e-stop (active LOW)

// Ultrasonic front sensor pins
#define USF_TRIG_PIN 10
#define USF_ECHO_PIN 11

// Ultrasonic side sensor pins
#define USS_TRIG_PIN 12
#define USS_ECHO_PIN 13

// Limits
float MAX_LINEAR = 0.25;   // m/s (informational)
float MAX_ANGULAR = 1.0;   // rad/s (informational)

// Simple mapping parameters
const int PWM_MAX = 255;

unsigned long last_us_time = 0;
const unsigned long US_SAMPLE_MS = 200; // sample ultrasonic sensors every 200ms

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);

  pinMode(ESTOP_PIN, INPUT_PULLUP);

  // Ultrasonic pins
  pinMode(USF_TRIG_PIN, OUTPUT);
  pinMode(USF_ECHO_PIN, INPUT);
  pinMode(USS_TRIG_PIN, OUTPUT);
  pinMode(USS_ECHO_PIN, INPUT);

  Serial.println("Nano controller ready");
}

// Read ultrasonic sensor (HC-SR04 style). Returns distance in cm or -1.0 on timeout
float read_ultrasonic_cm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL); // timeout 30ms
  if (duration == 0) return -1.0;
  float dist_cm = (float)duration / 58.0; // typical conversion
  return dist_cm;
}

// Convert a target speed in [-1,1] to PWM and direction
void write_motor(int pwm_pin, int dir_pin, float speed_norm) {
  // speed_norm is -1..1 ; negative -> reverse
  bool dir = speed_norm >= 0;
  float mag = fabs(speed_norm);
  mag = min(1.0f, mag);
  int pwm = (int)(mag * PWM_MAX);
  digitalWrite(dir_pin, dir ? HIGH : LOW);
  analogWrite(pwm_pin, pwm);
}

void stop_motors() {
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
}

void loop() {
  static String line;

  // Read a line from serial when available
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      // Process the line
      line.trim();
      // E-stop check
      if (digitalRead(ESTOP_PIN) == LOW) {
        Serial.println("E-STOP: pressed -> stopping");
        stop_motors();
        line = "";
        return;
      }

      // Parse L:...;A:...
      float lin = 0.0;
      float ang = 0.0;
      int iL = line.indexOf("L:");
      int iA = line.indexOf("A:");
      int semi = line.indexOf(';');
      if (iL >= 0 && semi > iL) {
        String sL = line.substring(iL + 2, semi);
        lin = atof(sL.c_str());
      }
      if (iA >= 0) {
        String sA = line.substring(iA + 2);
        ang = atof(sA.c_str());
      }

      // Safety limits (clamp)
      if (lin > MAX_LINEAR) lin = MAX_LINEAR;
      if (lin < -MAX_LINEAR) lin = -MAX_LINEAR;
      if (ang > MAX_ANGULAR) ang = MAX_ANGULAR;
      if (ang < -MAX_ANGULAR) ang = -MAX_ANGULAR;

      // Convert linear/angular to differential wheel commands (normalized)
      // very simple mix: left = lin - ang*factor ; right = lin + ang*factor
      float factor = 0.5; // tuning parameter (depends on wheelbase)
      float left = lin - ang * factor;
      float right = lin + ang * factor;

      // Normalize if out of range
      float maxv = max(fabs(left), fabs(right));
      if (maxv > MAX_LINEAR) {
        left = left / maxv * MAX_LINEAR;
        right = right / maxv * MAX_LINEAR;
      }

      // Convert to normalized [-1,1] for motor writing
      float left_norm = left / MAX_LINEAR;
      float right_norm = right / MAX_LINEAR;

      write_motor(LEFT_PWM, LEFT_DIR, left_norm);
      write_motor(RIGHT_PWM, RIGHT_DIR, right_norm);

      // Telemetry (optional)
      Serial.print("CMD L="); Serial.print(lin, 3);
      Serial.print(" A="); Serial.println(ang, 3);

      line = ""; // clear
    } else {
      line += c;
      // protect against runaway buffer
      if (line.length() > 200) line = "";
    }
  }

  // Periodically sample ultrasonic sensors and publish telemetry
  unsigned long now = millis();
  if (now - last_us_time >= US_SAMPLE_MS) {
    last_us_time = now;
    float usf = read_ultrasonic_cm(USF_TRIG_PIN, USF_ECHO_PIN);
    float uss = read_ultrasonic_cm(USS_TRIG_PIN, USS_ECHO_PIN);
    // Print telemetry in compact machine format: USF:<cm>;USS:<cm>
    // Use -1 for timeout/no return
    Serial.print("USF:");
    if (usf < 0) Serial.print("-1"); else Serial.print(usf, 1);
    Serial.print(";USS:");
    if (uss < 0) Serial.print("-1"); else Serial.print(uss, 1);
    Serial.print('\n');
  }
}

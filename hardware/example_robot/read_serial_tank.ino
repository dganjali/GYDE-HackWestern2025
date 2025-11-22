// read_serial_tank.ino
// Example Arduino sketch: parse camera commands and drive two motors (tank drive)
// Also runs an ultrasonic sensor and prints distance as "US <meters>" lines.

// Motor control pins (example - adapt to your wiring)
#define RPWM 5
#define LPWM 6
#define R_IN1 7
#define R_IN2 8
#define L_IN1 9
#define L_IN2 10

// Ultrasonic pins
#define TRIG_PIN 11
#define ECHO_PIN 12

// Simple proportional constant mapping dx -> motor differential
#define KP 0.8
#define TURN_SCALE 2.0

void setup() {
  Serial.begin(115200);

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // stop motors
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

// basic clamp
int clamp_i(int v, int lo, int hi){
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void setMotor(int pwmPin, int in1, int in2, int val){
  // val: -255..255
  if (val == 0){
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, 0);
  } else if (val > 0){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwmPin, clamp_i(val, 0, 255));
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwmPin, clamp_i(-val, 0, 255));
  }
}

float readUltrasonic(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1.0;
  float distance_cm = (duration / 2.0) / 29.1;
  return distance_cm / 100.0;
}

void loop(){
  // Read serial line if available
  if (Serial.available()){
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    // Expected format from camera bridge: dx:<n>;dy:<n>;F:<n>\n
    int dx = 0;
    int dy = 0;
    int fire = 0;

    int aIndex = line.indexOf("dx:");
    int eIndex = line.indexOf("dy:");
    int fIndex = line.indexOf("F:");
    int s1 = line.indexOf(';');
    int s2 = -1;
    if (s1 != -1) s2 = line.indexOf(';', s1+1);

    if (aIndex != -1 && eIndex != -1 && fIndex != -1){
      String ax = line.substring(aIndex+3, s1);
      String ay = line.substring(eIndex+3, s2);
      String af = line.substring(fIndex+2);
      dx = ax.toInt();
      dy = ay.toInt();
      fire = af.toInt();

      // map dx to differential motor command
      // err = -dx (camera convention), proportional
      float err = - (float)dx;
      float turn_out = KP * err;
      int diff = (int)(turn_out * TURN_SCALE);
      diff = clamp_i(diff, -255, 255);

      int left = diff;
      int right = -diff;

      setMotor(LPWM, L_IN1, L_IN2, left);
      setMotor(RPWM, R_IN1, R_IN2, right);

      // echo debug
      Serial.print("CMD L="); Serial.print(left);
      Serial.print(" R="); Serial.println(right);
    }
  }

  // Periodically send ultrasonic reading
  static unsigned long last_us = 0;
  unsigned long now = millis();
  if (now - last_us > 150){
    last_us = now;
    float d = readUltrasonic();
    char buf[32];
    dtostrf(d, 0, 3, buf);
    Serial.print("US "); Serial.print(buf); Serial.print("\n");
  }
}

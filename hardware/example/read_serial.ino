#include <Servo.h>

Servo myServo;
int currentPos = 90;
#define kP 0.6

// Motor control pins
#define RPWM 5
#define LPWM 6
#define R_EN 7
#define L_EN 8
#define servoPin 9
#define kickerPin 10

// Button pin
#define buttonPin 4
bool systemEnabled = false;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);

  // Motor setup
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

  // Button setup
  pinMode(buttonPin, INPUT_PULLUP);  // active LOW
}

void loop() {
  // Wait for a one-time button press
  if (!systemEnabled) {
    if (digitalRead(buttonPin) == LOW) {
      delay(50);  // debounce
      if (digitalRead(buttonPin) == LOW) {
        systemEnabled = true;
        Serial.println("System Enabled");
      }
    }
    return; // Exit loop() until systemEnabled is true
  }

  // dx: 3; 
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');

    int dx = 90;
    int dy = 0;
    int fire = 0;
    double pitch = 0;

    int aIndex = input.indexOf("dx:");
    int eIndex = input.indexOf("dy:");
    int pIndex = input.indexOf("pitch:");
    int fIndex = input.indexOf("F:");

    int semicolon1 = input.indexOf(';');
    int semicolon2 = input.indexOf(';', semicolon1 + 1);
    int semicolon3 = input.indexOf(';', semicolon2 + 1);

    if (aIndex != -1 && eIndex != -1 && fIndex != -1) {
      String aValue = input.substring(aIndex + 3, semicolon1);
      dx = aValue.toInt();

      String eValue = input.substring(eIndex + 3, semicolon2);
      dy = eValue.toInt();

      String pValue = input.substring(pIndex + 6, semicolon3);
      pitch = pValue.toInt();

      String fValue = input.substring(fIndex + 2);
      fire = fValue.toInt();

      Serial.print("dx: ");
      Serial.println(dx);
      Serial.print("dy: ");
      Serial.println(dy);
      Serial.print("pitch: ");
      Serial.println(pitch);
      Serial.print("Fire: ");
      Serial.println(fire);
      // double error = (kP*dx + 180)/2;

      myServo.write((kP*dx + 180)/2);
      
    }
  }
}

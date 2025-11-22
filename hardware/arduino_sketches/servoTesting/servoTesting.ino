#include <Servo.h>

Servo continuousServo; // pin 9
Servo standardServo;   // pin 10

#define buttonPin 4
bool testStarted = false;

void setup() {
  continuousServo.attach(9);  
  standardServo.attach(10);   //180 deg servo

  pinMode(buttonPin, INPUT); 

  Serial.begin(9600);
  Serial.println("Ready. Press button on pin 4 to run test once.");
}

void loop() {
  if (!testStarted && digitalRead(buttonPin) == HIGH) {
    delay(50); 
    if (digitalRead(buttonPin) == HIGH) {
      testStarted = true;
      Serial.println("Button pressed. Running test...");
    }
  }

  if (testStarted) {
    // --- Standard servo sweep ---
    // Serial.println("0");
    // standardServo.write(0);
    // delay(1000);

    // Serial.println("90");
    // standardServo.write(90);
    // delay(1000);

    // Serial.println("180");
    // standardServo.write(180);
    // delay(1000);

    Serial.println("other servo forward");
    continuousServo.write(130);  
    delay(500);

    // Serial.println("other servo stop");
    // continuousServo.write(90);   
    // delay(1000);

    // Serial.println("other servo reverse");
    // continuousServo.write(40);   
    // delay(500);

    // Serial.println("other servo stop");
    // continuousServo.write(90);   
    // delay(1000);

    testStarted = false;

  }
}

// Pin Definitions
const int RPWM = 5;   
const int LPWM = 6;   
const int R_EN = 7;   
const int L_EN = 8;   

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);


  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
}

void loop() {

  analogWrite(RPWM, 0);  
  analogWrite(LPWM, 200);
  delay(3000);

  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  delay(1000);

}

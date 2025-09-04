const int motorPWM = 11; const int motorDir = 10;

void setup() {
  Serial.begin(9600);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);
}

void loop() {
  digitalWrite(motorDir, LOW);
  analogWrite(motorPWM, 35);
  delay(5000);
  digitalWrite(motorDir, HIGH);
  analogWrite(motorPWM, 20);
  delay(5000);
}
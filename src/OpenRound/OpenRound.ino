#include <Servo.h>

Servo steering;
const int servoPin = 9;
const int motorPWM = 11;
const int motorDir = 10;
const int trigLeft = A5;
const int echoLeft = A4;
const int trigRight = A3;
const int echoRight = A2;
const int trigCenter = A1;
const int echoCenter = A0;

void setup() {
  steering.attach(servoPin);
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);
  pinMode(trigCenter, OUTPUT);
  pinMode(echoCenter, INPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);
}

void loop() {
  float distLeft = readUltrasonic(trigLeft, echoLeft);
  float distCenter = readUltrasonic(trigCenter, echoCenter);
  float distRight = readUltrasonic(trigRight, echoRight);

  digitalWrite(motorDir, LOW);
  analogWrite(motorPWM, 30);

  if (distRight>=200) {
    if (distCenter<=30) {
      analogWrite(motorPWM, 20);
      steering.write(135);
      delay(300);
      steering.write(90);
      analogWrite(motorPWM, 30);
    }    
  }
  else if (distLeft>=200) {
    if (distCenter<=30) {
      analogWrite(motorPWM, 20);
      steering.write(45);
      delay(300);
      steering.write(90);
      analogWrite(motorPWM, 30);
    }  
  }
}

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(20);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(100);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2.0;
}

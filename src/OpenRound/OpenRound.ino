#include <Servo.h>

// Ultrasonic sensor pins

Servo steering;  // Create servo object
const int servoPin = 9;
const int motorPWM = 11;  // PWM control
const int motorDir = 10;  // Direction control
const int trigLeft = A5;
const int echoLeft = A4;
const int trigRight = A3;
const int echoRight = A2;
const int trigCenter = A1;
const int echoCenter = A0;

void setup() {
  // Serial.begin(9600);
  steering.attach(servoPin);
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);
  pinMode(trigCenter, OUTPUT);
  pinMode(echoCenter, INPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);

  digitalWrite(motorDir, LOW);
  analogWrite(motorPWM, 180);
}

void loop() {
  float distLeft = readUltrasonic(trigLeft, echoLeft);
  float distCenter = readUltrasonic(trigCenter, echoCenter);
  float distRight = readUltrasonic(trigRight, echoRight);

  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input=="B") {
      if (distLeft > 30 && distCenter > 30 && distRight > 30) {
        analogWrite(motorPWM, 30);
        steering.write(90);
        delay(20);
      }
      else if (distLeft > 30 && distCenter > 30 && distRight < 30) {
        analogWrite(motorPWM, 28);
        steering.write(45);
        delay(150);
        steering.write(90);
        delay(25);
      }
      else if (distLeft > 30 && distCenter < 30 && distRight > 30) {
        analogWrite(motorPWM, 28);
        steering.write(45);
        delay(150);
        steering.write(90);
        delay(25);
      }
      else if (distLeft > 30 && distCenter < 30 && distRight < 30) {
        analogWrite(motorPWM, 28);
        steering.write(45);
        delay(150);
        steering.write(90);
        delay(25);
      }
      else if (distLeft < 30 && distCenter > 30 && distRight > 30) {
        analogWrite(motorPWM, 28);
        steering.write(135);
        delay(150);
        steering.write(90);
        delay(25);
      }
      else if (distLeft < 30 && distCenter > 30 && distRight < 25) {
        analogWrite(motorPWM, 28);
        steering.write(45);
        delay(150);
        steering.write(90);
        delay(25);
      }
      else if (distLeft < 30 && distCenter < 30 && distRight > 30) {
        analogWrite(motorPWM, 28);
        steering.write(135);
        delay(150);
        steering.write(90);
        delay(25);
      }
      else if (distLeft < 30 && distCenter < 30 && distRight < 30) {
        analogWrite(motorPWM, 28);
        steering.write(45);
        delay(150);
        steering.write(90);
        delay(25);
      }
    }
    if (input=="O") {
      if (distLeft > 30 && distCenter > 30 && distRight > 30) {
        analogWrite(motorPWM, 30);
        steering.write(90);
        delay(20);
      }
      else if (distLeft > 30 && distCenter > 30 && distRight < 30) {
        analogWrite(motorPWM, 28);
        steering.write(45);
        delay(150);
        steering.write(90);
        delay(25);
      }
      else if (distLeft > 30 && distCenter < 30 && distRight > 30) {
        analogWrite(motorPWM, 28);
        steering.write(135);
        delay(150);
        steering.write(90);
        delay(25);
      }
      else if (distLeft > 30 && distCenter < 30 && distRight < 30) {
        analogWrite(motorPWM, 28);
        steering.write(135);
        delay(150);
        steering.write(90);
        delay(25);
      }
      else if (distLeft < 30 && distCenter > 30 && distRight > 30) {
        analogWrite(motorPWM, 28);
        steering.write(135);
        delay(150);
        steering.write(90);
        delay(25);
      }
      else if (distLeft < 30 && distCenter > 30 && distRight < 25) {
        analogWrite(motorPWM, 28);
        steering.write(135);
        delay(150);
        steering.write(90);
        delay(25);
      }
      else if (distLeft < 30 && distCenter < 30 && distRight > 30) {
        analogWrite(motorPWM, 28);
        steering.write(135);
        delay(150);
        steering.write(90);
        delay(25);
      }
      else if (distLeft < 30 && distCenter < 30 && distRight < 30) {
        analogWrite(motorPWM, 28);
        steering.write(135);
        delay(150);
        steering.write(90);
        delay(25);
      }
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

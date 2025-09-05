#include <Servo.h>

Servo steering;
const int servoPin = 9;
const int motorPWM = 11;
const int motorDir = 10; 
const int trigLeft = A5; const int echoLeft = A4;
const int trigRight = A3; const int echoRight = A2;
const int trigCenter = A1; const int echoCenter = A0;

void setup() {
  Serial.begin(9600);
  steering.attach(servoPin);
  pinMode(trigLeft, OUTPUT); pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);
  pinMode(trigCenter, OUTPUT); pinMode(echoCenter, INPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir, OUTPUT);
  while (!Serial);
  Serial.println("Connected to Raspberry Pi");
}

int readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(20);
  digitalWrite(trigPin, HIGH); delayMicroseconds(100);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void loop() {
  int distLeft = readUltrasonic(trigLeft, echoLeft);
  int distCenter = readUltrasonic(trigCenter, echoCenter);
  int distRight = readUltrasonic(trigRight, echoRight);

  Serial.print("Left: "); Serial.print(distLeft); Serial.print(" cm\t");
  Serial.print("Center: "); Serial.print(distCenter); Serial.print(" cm\t");
  Serial.print("Right: "); Serial.print(distRight); Serial.println(" cm");

  digitalWrite(motorDir, LOW);
  analogWrite(motorPWM, 40);

  if (Serial.available() > 0) {
    char input = Serial.read();

    if (input == 'B') {
      if (distLeft >= 70) {
        steering.write(45);
        delay(1200);
        steering.write(90);
      } 
      else {
        // slow down motor gradually
        for (int i = 60; i > 39; i--) {
          analogWrite(motorPWM, i);
          delay(20);
        }

        if (distLeft <= 20) {
          analogWrite(motorPWM, 30);   // corrected
          steering.write(115);
          delay(150);
          steering.write(55);
          delay(100);
          analogWrite(motorPWM, 60);   // corrected
        } 
        else if (distRight <= 20) {
          analogWrite(motorPWM, 30);   // corrected
          steering.write(65);
          delay(150);
          steering.write(125);
          delay(100);
          analogWrite(motorPWM, 60);   // corrected
        }
      }
    }

    else if (input == 'O') {
      if (distRight >= 70) {
        steering.write(135);
        delay(1200);
        steering.write(90);
      } 
      else {
        // slow down motor gradually
        for (int i = 60; i > 39; i--) {
          analogWrite(motorPWM, i);
          delay(20);
        }

        if (distLeft <= 20) {
          analogWrite(motorPWM, 30);   // corrected
          steering.write(115);
          delay(150);
          steering.write(55);
          delay(100);
          analogWrite(motorPWM, 60);   // corrected
        } 
        else if (distRight <= 20) {
          analogWrite(motorPWM, 30);   // corrected
          steering.write(65);
          delay(150);
          steering.write(125);
          delay(100);
          analogWrite(motorPWM, 60);   // corrected
        }
      }
    }
  }
}

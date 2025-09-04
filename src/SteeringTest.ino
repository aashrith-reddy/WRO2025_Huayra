#include <Servo.h>
Servo steering;
const int servoPin = 9;
void setup() {
  Serial.begin(9600);
  steering.attach(servoPin);
}

void loop() {
  steering.write(0);
  delay(500);
  steering.write(90);
  delay(500);
  steering.write(180);
  delay(500);
}
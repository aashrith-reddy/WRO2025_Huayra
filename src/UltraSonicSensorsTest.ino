const int trigLeft = 2; const int echoLeft = 3;
const int trigRight = A2; const int echoRight = A3;
const int trigCenter = A1; const int echoCenter = A0;

void setup() {
  Serial.begin(9600);
  pinMode(trigLeft, OUTPUT); pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);
  pinMode(trigCenter, OUTPUT); pinMode(echoCenter, INPUT);
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
}
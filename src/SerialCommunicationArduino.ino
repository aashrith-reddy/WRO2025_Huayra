void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Connected to Raspberry Pi");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    Serial.print("Got: ");
    Serial.println(c);
  }

  static unsigned long lastTime = 0;
  if (millis() - lastTime > 2000) {
    Serial.println("90201-TravisScott");
    lastTime = millis();
  }
}

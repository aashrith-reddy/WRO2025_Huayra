void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud
}

void loop() {
  // Check if data is available from Raspberry Pi
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Read command from Pi

    // Example: control an LED on pin 13
    if (command == "MAGENTA") {
      Serial.println("The color Magenta has been detected");
    }
    else if (command == "RED") {
      Serial.println("The color Red has been detected");
    }
    else {
      Serial.println("Unknown command: " + command);
    }
  }
}
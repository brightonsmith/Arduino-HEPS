void setup() {
  Serial.begin(9600);  // Initialize Serial communication at 9600 baud rate
}

void loop() {
  if (Serial.available()) {
    String message = Serial.readString();  // Read data from ESP32
    Serial.println("Message Received: ");
    Serial.println(message);  // Print received message
  }
}

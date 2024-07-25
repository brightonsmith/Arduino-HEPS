#include <HardwareSerial.h>

HardwareSerial SerialPort(2); // use UART2

void setup() {
  Serial.begin(115200);  // Initialize Serial communication at 115200 baud rate for debugging
  SerialPort.begin(9600, SERIAL_8N1, 16, 17);   // Initialize Serial2 communication at 9600 baud rate
}

void loop() {
  SerialPort.println("Hello Arduino");  // Send data to Arduino via UART2
  delay(1500);  // Delay for 1.5 seconds
}

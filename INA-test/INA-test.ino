#include <Adafruit_INA260.h>

Adafruit_INA260 ina1 = Adafruit_INA260();
Adafruit_INA260 ina2 = Adafruit_INA260();

void setup() {
  Serial.begin(115200);
  // Wait until serial port is opened
  while (!Serial) { delay(10); }

  Serial.println("Adafruit INA260 Test");

  if (!ina1.begin(0x40)) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Found INA260 chip");

  if (!ina2.begin(0x41)) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  }
  Serial.println("Found INA260 chip");
}

void loop() {
  Serial.print("Battery current: ");
  Serial.print(ina1.readCurrent() / 1000.0);
  Serial.println(" A");

  Serial.print("Battery voltage: ");
  Serial.print(ina1.readBusVoltage() / 1000.0);
  Serial.println(" V");

  Serial.print("Battery power: ");
  Serial.print(ina1.readPower() / 1000.0);
  Serial.println(" W");

  Serial.print("Alternator current: ");
  Serial.print(ina2.readCurrent() / 1000.0);
  Serial.println(" A");

  Serial.print("Alternator voltage: ");
  Serial.print(ina2.readBusVoltage() / 1000.0);
  Serial.println(" V");

  Serial.print("Alternator power: ");
  Serial.print(ina2.readPower() / 1000.0);
  Serial.println(" W");

  Serial.println();
  delay(1000);
}
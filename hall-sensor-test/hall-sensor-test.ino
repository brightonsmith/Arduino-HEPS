// Define the ADC pins connected to your hall effect sensor
const int voltagePin = 33; // Example GPIO pin for voltage sense
const int currentPin = 27; // Example GPIO pin for current sense

const float ADC_MAX = 4095.0; // 12-bit ADC for ESP32
const float VREF = 3.3; // Reference voltage for ESP32 ADC

// Conversion factors based on the sensor's datasheet
const float VOLTAGE_SCALE = 69.0 / VREF; // 69V max voltage corresponding to 3.3V ADC input
const float CURRENT_SENSITIVITY = 0.02; // 20mV/A or 0.02V/A

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // 12-bit resolution for ESP32 ADC
}

void loop() {
  // Read ADC values
  int voltageADC = analogRead(voltagePin);
  int currentADC = analogRead(currentPin);

  // Convert ADC values to actual voltage and current
  float voltage = ((voltageADC / ADC_MAX) * VREF * VOLTAGE_SCALE);
  float current = ((currentADC / ADC_MAX) * VREF / CURRENT_SENSITIVITY);
  float power = voltage * current;

  // Print the results to Serial Monitor
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");

  Serial.print("Current: ");
  Serial.print(current);
  Serial.println(" A");

  Serial.print("Power: ");
  Serial.print(power);
  Serial.println(" W");

  delay(50);
}

#include <ESP32Servo.h> // ESP32Servo library installed by Library Manager
#include "ESC.h" // RC_ESP library installed by Library Manager

#define ESC_PIN 13 // connected to ESC control wire
#define LED_PIN 2 // not defaulted properly for ESP32s/you must define it

// Note: the following speeds may need to be modified for your particular hardware.
#define MIN_SPEED 1040 // speed just slow enough to turn motor off
#define MAX_SPEED 1450 // speed where my motor drew 3.6 amps at 12v.

ESC myESC(ESC_PIN, 1000, 1850, 500); // ESC_Name (PIN, Minimum Value, Maximum Value, Arm Value)

long int val; // variable to read the value from the analog pin

void setup() {
  Serial.begin(9600);
  delay(1000);
  pinMode(ESC_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // set led to on to indicate arming
  myESC.arm(); // Send the Arm command to ESC
  delay(5000); // Wait a while
  digitalWrite(LED_PIN, LOW); // led off to indicate arming completed

  // the following loop turns on the motor slowly, so get ready
  for (int i = 0; i < 350; i++) { // run speed from 840 to 1190
    myESC.speed(MIN_SPEED - 200 + i); // motor starts up about half way through loop
    delay(10);
  }
  // After the initial ramp-up, set the motor to a constant speed
  myESC.speed((MIN_SPEED + MAX_SPEED) / 2); // set to a constant speed in the middle of the range
}

void loop() {
  // Keep the motor running at the constant speed set in setup
  // Alternatively, you can set another speed or use some logic to vary the speed
  myESC.speed((MIN_SPEED + MAX_SPEED) / 2); // set to a constant speed in the middle of the range
  delay(100); // add a short delay to avoid overwhelming the ESC
}

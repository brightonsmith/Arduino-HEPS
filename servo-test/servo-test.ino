#include <ESP32Servo.h>

static const int servoPin = 13;

Servo servo1;

void setup() {

  Serial.begin(115200);
  servo1.attach(servoPin);
}

void loop() {
  for(int posDegrees = 40; posDegrees <= 160; posDegrees++) {
    servo1.write(posDegrees);
    Serial.println(posDegrees);
    delay(5);
  }

  for(int posDegrees = 160; posDegrees >= 40; posDegrees--) {
    servo1.write(posDegrees);
    Serial.println(posDegrees);
    delay(5);
  }
  //servo1.write(40);
}
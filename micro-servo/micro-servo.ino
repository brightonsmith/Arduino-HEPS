#include <Servo.h>

Servo myServo;  // create a servo object
int servoPin = 9;  // define the pin the servo is connected to

void setup() {
  myServo.attach(servoPin);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  // Move the servo to 0 degrees
  myServo.write(0);
  delay(1000);  // wait for 1 second

  // Move the servo to 90 degrees
  myServo.write(90);
  delay(1000);  // wait for 1 second

  // Move the servo to 180 degrees
  myServo.write(180);
  delay(1000);  // wait for 1 second

  // Move the servo back to 90 degrees
  myServo.write(90);
  delay(1000);  // wait for 1 second
}

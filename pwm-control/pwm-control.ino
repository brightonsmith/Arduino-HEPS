#include <WiFi.h>
#include <ESP32Servo.h>
#include "ESC.h"

/////////////////////////////////////////////////////////////////////////////Config and Globals////////////////////////////////////////////////////////////////////////////////////////////
// Macros
#define LED_PIN 2
#define ESC_PIN 13
#define SERVO_PIN 14 
#define MIN_SPEED 1100
#define MAX_SPEED 1850
#define ARM_SPEED 1050
#define MIN_ANGLE 30
#define MAX_ANGLE 160

// WiFi credentials
const char *ssid = "Crazy Frog"; // Set by user
const char *password = "Stravinsky1913"; // Set by user

// Web server
WiFiServer server(80);

// ESC
ESC esc(ESC_PIN, MIN_SPEED, MAX_SPEED, ARM_SPEED);

int esc25Value = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * 0.25;
int esc50Value = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * 0.50;
int esc75Value = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * 0.75;
String esc25ValueStr = String(esc25Value);
String esc50ValueStr = String(esc50Value);
String esc75ValueStr = String(esc75Value);
String minSpeedStr = String(MIN_SPEED);
String maxSpeedStr = String(MAX_SPEED);

// Servo
Servo servo;

int servo25Value = MIN_ANGLE + (MAX_ANGLE - MIN_ANGLE) * 0.25;
int servo50Value = MIN_ANGLE + (MAX_ANGLE - MIN_ANGLE) * 0.50;
int servo75Value = MIN_ANGLE + (MAX_ANGLE - MIN_ANGLE) * 0.75;
String servo25ValueStr = String(servo25Value);
String servo50ValueStr = String(servo50Value);
String servo75ValueStr = String(servo75Value);
String minAngleStr = String(MIN_ANGLE);
String maxAngleStr = String(MAX_ANGLE);

// HTTP
// Variable to store the HTTP request
String header;

// Decode HTTP GET value
String escString = String(5);
String servoString = String(5);
int pos1 = 0;
int pos2 = 0;

// ESC and Servo control variables
int escValue = 0;
int servoValue = 0;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds
const long timeoutTime = 2000;

////////////////////////////////////////////////////////////////////////////////////////Setup////////////////////////////////////////////////////////////////////////////////////////////

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

// Arduino setup function
void setup() {
  Serial.begin(9600);
  delay(1000);

  // ESC
  pinMode(ESC_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // set led to on to indicate arming
  esc.arm(); // Send the Arm command to ESC

  // Servo
  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	servo.setPeriodHertz(50);    // standard 50 hz servo
	servo.attach(SERVO_PIN, 1000, 2000); // attaches the servo on pin 18 to the servo object

  delay(5000); // Wait a while
  digitalWrite(LED_PIN, LOW); // led off to indicate arming completed

  setup_wifi();
}

////////////////////////////////////////////////////////////////////////////////////////Loop/////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // Print a message out in the serial port
    String currentLine = "";                // Make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // Loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // If there's bytes to read from the client,
        char c = client.read();             // Read a byte, then
        Serial.write(c);                    // Print it out the serial monitor
        header += c;
        if (c == '\n') {                    // If the byte is a newline character
          // If the current line is blank, you got two newline characters in a row.
          // That's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // HTML content
            client.println("<!DOCTYPE html><html lang=\"en\"><head>");
            client.println("<meta charset=\"UTF-8\">");
            client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
            client.println("<style>");
            client.println("body { display: flex; justify-content: center; align-items: center; height: 100vh; font-family: Arial, sans-serif; }");
            client.println(".container-wrapper { width: 100%; max-width: 600px; text-align: center; }");
            client.println(".container { margin-bottom: 20px; }");
            client.println(".slider { width: 100%; max-width: 300px; }");
            client.println(".button { display: inline-block; padding: 10px 20px; font-size: 20px; cursor: pointer; text-align: center; text-decoration: none; outline: none; color: #fff; background-color: #4CAF50; border: none; border-radius: 15px; box-shadow: 0 9px #999; }");
            client.println(".button:hover { background-color: #3e8e41; }");
            client.println(".button:active { background-color: #3e8e41; box-shadow: 0 5px #666; transform: translateY(4px); }");
            client.println("#escKillButton, #servoKillButton { background-color: red; color: white; padding: 20px 40px; font-size: 24px; margin-top: 20px; }");
            client.println("hr { border: 1px solid #ccc; margin: 20px 0; }");
            client.println("@media screen and (orientation: portrait) { .container-wrapper { max-width: 600px; } }");
            client.println("@media screen and (orientation: landscape) { .container-wrapper { display: flex; justify-content: space-between; align-items: flex-start; max-width: 1200px; } .container { width: 80%; margin-bottom: 0; } hr { display: none; } .container-wrapper::before { content: \"\"; display: block; width: 1px; height: 100%; background-color: #ccc; } }");
            client.println("</style></head><body>");

            // ESC Control Center
            client.println("<div class=\"container-wrapper\">");
            client.println("<div class=\"container\">");
            client.println("<h1>ESC Control Center</h1>");
            client.println("<p>Pulse Width: <span id=\"escPulse\">" + minSpeedStr + "</span>&#956;s</p>");
            client.println("<input type=\"range\" min=\"" + minSpeedStr + "\" max=\"" + maxSpeedStr + "\" class=\"slider\" id=\"escSlider\" onchange=\"esc(this.value)\" value=\""+escString+"\"/>");
            client.println("<br/><br/>");
            client.println("<button class=\"button\" id=\"escUpButton\">Up</button>");
            client.println("<button class=\"button\" id=\"escDownButton\">Down</button>");
            client.println("<br/><br/>");
            client.println("<button class=\"button\" id=\"escButton_25\" style=\"background-color: green; color: white;\">25%</button>");
            client.println("<button class=\"button\" id=\"escButton_50\" style=\"background-color: green; color: white;\">50%</button>");
            client.println("<button class=\"button\" id=\"escButton_75\" style=\"background-color: green; color: white;\">75%</button>");
            client.println("<button class=\"button\" id=\"escMaxButton\" style=\"background-color: green; color: white;\">MAX</button>");
            client.println("<br/><br/>");
            client.println("<button class=\"button\" id=\"escKillButton\">KILL</button>");
            client.println("<audio id=\"killSound\" src=\"https://audio.jukehost.co.uk/WZouY1JxlTfazBV9wwZeHlW60jgBsGVa\"></audio>");
            client.println("</div>");
            client.println("<hr>");

            // Servo Control Center
            client.println("<div class=\"container\"><h1>Servo Control Center</h1>");
            client.println("<p>Angle: <span id=\"servoAngle\">" + minAngleStr + "</span>&#176;</p>");
            client.println("<input type=\"range\" min=\"" + minAngleStr + "\" max=\"" + maxAngleStr + "\" class=\"slider\" id=\"servoSlider\" onchange=\"servo(this.value)\" value=\""+servoString+"\"/>");
            client.println("<br/><br/><button class=\"button\" id=\"servoUpButton\" style=\"background-color: blue; color: white;\">Up</button>");
            client.println("<button class=\"button\" id=\"servoDownButton\" style=\"background-color: blue; color: white;\">Down</button><br/><br/>");
            client.println("<button class=\"button\" id=\"servoButton_25\" style=\"background-color: blue; color: white;\">25%</button>");
            client.println("<button class=\"button\" id=\"servoButton_50\" style=\"background-color: blue; color: white;\">50%</button>");
            client.println("<button class=\"button\" id=\"servoButton_75\" style=\"background-color: blue; color: white;\">75%</button>");
            client.println("<button class=\"button\" id=\"servoMaxButton\" style=\"background-color: blue; color: white;\">MAX</button>");
            client.println("<br/><br/>");
            client.println("<button class=\"button\" id=\"servoKillButton\">KILL</button>");
            client.println("<audio id=\"killSound\" src=\"https://audio.jukehost.co.uk/WZouY1JxlTfazBV9wwZeHlW60jgBsGVa\"></audio>");
            client.println("</div>");
            client.println("</div>");
            client.println("</body>");

            // JavaScript
            client.println("<script src=\"https://ajax.googleapis.com/ajax/libs/jquery/3.3.1/jquery.min.js\"></script>");
            client.println("<script>var escSlider=document.getElementById(\"escSlider\");var escP=document.getElementById(\"escPulse\");escP.innerHTML=escSlider.value;");
            client.println("escSlider.oninput = function() { escP.innerHTML = this.value; esc(this.value); }");
            client.println("$.ajaxSetup({timeout:1000}); function esc(pulse) { ");
            client.println("$.get(\"/?escValue=\" + pulse + \"&\"); {Connection: close};}");
            client.println("document.getElementById(\"escUpButton\").addEventListener(\"click\", function() { adjustESC(25); });");
            client.println("document.getElementById(\"escDownButton\").addEventListener(\"click\", function() { adjustESC(-25); });");
            client.println("document.getElementById(\"escButton_25\").addEventListener(\"click\", function() { escSlider.value = " + esc25ValueStr + "; escP.innerHTML = " + esc25ValueStr + "; esc(" + esc25ValueStr + "); });");
            client.println("document.getElementById(\"escButton_50\").addEventListener(\"click\", function() { escSlider.value = " + esc50ValueStr + "; escP.innerHTML = " + esc50ValueStr + "; esc(" + esc50ValueStr + "); });");
            client.println("document.getElementById(\"escButton_75\").addEventListener(\"click\", function() { escSlider.value = " + esc75ValueStr + "; escP.innerHTML = " + esc75ValueStr + "; esc(" + esc75ValueStr + "); });");
            client.println("document.getElementById(\"escMaxButton\").addEventListener(\"click\", function() { escSlider.value = " + maxSpeedStr + "; escP.innerHTML = " + maxSpeedStr + "; esc(" + maxSpeedStr + "); });");
            client.println("document.getElementById(\"escKillButton\").addEventListener(\"click\", function() { escSlider.value = " + minSpeedStr + "; escP.innerHTML = " + minSpeedStr + "; esc(" + minSpeedStr + "); var audio = document.getElementById('killSound'); audio.play(); });");
            client.println("function adjustESC(offset) { var newValue = parseInt(escP.innerHTML) + offset;");
            client.println("if (newValue < " + minSpeedStr + ") { newValue = " + minSpeedStr + "; }");
            client.println("if (newValue > " + maxSpeedStr + ") { newValue = " + maxSpeedStr + "; }");
            client.println("escSlider.value = newValue; escP.innerHTML = newValue; esc(newValue); }");

            client.println("var servoSlider=document.getElementById(\"servoSlider\");var servoA=document.getElementById(\"servoAngle\");servoA.innerHTML=servoSlider.value;");
            client.println("servoSlider.oninput = function() { servoA.innerHTML = this.value; servo(this.value); }");
            client.println("$.ajaxSetup({timeout:1000}); function servo(angle) { ");
            client.println("$.get(\"/?servoValue=\" + angle + \"&\"); {Connection: close};}");
            client.println("document.getElementById(\"servoUpButton\").addEventListener(\"click\",function(){adjustServo(5);});");
            client.println("document.getElementById(\"servoDownButton\").addEventListener(\"click\",function(){adjustServo(-5);});");
            client.println("document.getElementById(\"servoButton_25\").addEventListener(\"click\", function() { servoSlider.value = " + servo25ValueStr + "; servoA.innerHTML = " + servo25ValueStr + "; servo(" + servo25ValueStr + "); });");
            client.println("document.getElementById(\"servoButton_50\").addEventListener(\"click\", function() { servoSlider.value = " + servo50ValueStr + "; servoA.innerHTML = " + servo50ValueStr + "; servo(" + servo50ValueStr + "); });");
            client.println("document.getElementById(\"servoButton_75\").addEventListener(\"click\", function() { servoSlider.value = " + servo75ValueStr + "; servoA.innerHTML = " + servo75ValueStr + "; servo(" + servo75ValueStr + "); });");
            client.println("document.getElementById(\"servoMaxButton\").addEventListener(\"click\", function() { servoSlider.value = " + maxAngleStr + "; servoA.innerHTML = " + maxAngleStr + "; servo(" + maxAngleStr + "); });");
            client.println("document.getElementById(\"servoKillButton\").addEventListener(\"click\", function() { servoSlider.value = " + minAngleStr + "; servoA.innerHTML = " + minAngleStr + "; servo(" + minAngleStr + "); var audio = document.getElementById('killSound'); audio.play(); });");
            client.println("function adjustServo(offset) { var newValue = parseInt(servoA.innerHTML) + offset;");
            client.println("if (newValue < " + minAngleStr + ") { newValue = " + minAngleStr + "; }");
            client.println("if (newValue > " + maxAngleStr + ") { newValue = " + maxAngleStr + "; }");
            client.println("servoSlider.value = newValue; servoA.innerHTML = newValue; servo(newValue); }");
            client.println("</script></html>");

            // Check if the client request was for ESC
            if (header.indexOf("GET /?escValue=") >= 0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              escString = header.substring(pos1 + 1, pos2);

              // Set the ESC pulse width
              escValue = escString.toInt();
              // Call your ESC control function here
              esc.speed(escValue);

              Serial.println("ESC Value: " + String(escValue));
            }

            // Check if the client request was for Servo
            if (header.indexOf("GET /?servoValue=") >= 0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              servoString = header.substring(pos1 + 1, pos2);

              // Set the Servo angle
              servoValue = servoString.toInt();
              // Call your Servo control function here
              servo.write(servoValue);

              Serial.println("Servo Value: " + String(servoValue));
            }

            client.println();
            break;
          } else { // If you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // If you got anything else but a carriage return character,
          currentLine += c;      // Add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
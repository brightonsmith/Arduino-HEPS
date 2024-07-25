#include <WiFi.h>
#include <ESP32Servo.h>
#include "ESC.h"

/////////////////////////////////////////////////////////////////////////////Config and Globals////////////////////////////////////////////////////////////////////////////////////////////
// Macros
#define LED_PIN 2
#define ESC_PIN 13
#define SERVO_PIN 14 
#define MIN_SPEED 1000
#define MAX_SPEED 1700

// WiFi credentials
const char *ssid = "Crazy Frog"; // Set by user
const char *password = "Stravinsky1913"; // Set by user

// Web server
WiFiServer server(80);

// ESC
ESC esc(ESC_PIN, 1000, 1850, 500);

// Servo
Servo servo;


// HTTP
// Variable to store the HTTP request
String header;

// Decode HTTP GET value
String valueString = String(5);
int pos1 = 0;
int pos2 = 0;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds
const long timeoutTime = 2000;

int button25Value = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * 0.25;
int button50Value = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * 0.50;
int button75Value = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * 0.75;
String button25ValueStr = String(button25Value);
String button50ValueStr = String(button50Value);
String button75ValueStr = String(button75Value);
String minSpeedStr = String(MIN_SPEED);
String maxSpeedStr = String(MAX_SPEED);

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
  pinMode(ESC_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // set led to on to indicate arming
  esc.arm(); // Send the Arm command to ESC
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
            // Prepare the response
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            client.println("<!DOCTYPE html>");
            client.println("<html lang=\"en\">");
            client.println("<head>");
            client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
            client.println("<style>");
            client.println("body { display: flex; justify-content: center; align-items: center; height: 100vh; font-family: Arial, sans-serif; }");
            client.println(".container { text-align: center; }");
            client.println(".slider { width: 100%; max-width: 300px; }");
            client.println(".button { display: inline-block; padding: 10px 20px; font-size: 20px; cursor: pointer; text-align: center; text-decoration: none; outline: none; color: #fff; background-color: #4CAF50; border: none; border-radius: 15px; box-shadow: 0 9px #999; }");
            client.println(".button:hover { background-color: #3e8e41; }");
            client.println(".button:active { background-color: #3e8e41; box-shadow: 0 5px #666; transform: translateY(4px); }");
            client.println("#killButton { background-color: red; color: white; padding: 20px 40px; font-size: 24px; margin-top: 20px; }");
            client.println("</style>");
            client.println("</head>");
            client.println("<body>");
            client.println("<div class=\"container\">");
            client.println("<h1>ESC Control Center</h1>");
            client.println("<p>Pulse Width: <span id=\"escPulse\"></span>&#956;s</p>");
            client.println("<input type=\"range\" min=\"" + minSpeedStr + "\" max=\"" + maxSpeedStr + "\" class=\"slider\" id=\"escSlider\" onchange=\"esc(this.value)\" value=\""+valueString+"\"/>");
            client.println("<br/><br/>");
            client.println("<button class=\"button\" id=\"upButton\">Up</button>");
            client.println("<button class=\"button\" id=\"downButton\">Down</button>");
            client.println("<br/><br/>");
            client.println("<button class=\"button\" id=\"button_25\" style=\"background-color: green; color: white;\">25%</button>");
            client.println("<button class=\"button\" id=\"button_50\" style=\"background-color: green; color: white;\">50%</button>");
            client.println("<button class=\"button\" id=\"button_75\" style=\"background-color: green; color: white;\">75%</button>");
            client.println("<button class=\"button\" id=\"maxButton\" style=\"background-color: green; color: white;\">MAX</button>");
            client.println("<br/><br/>");
            client.println("<button class=\"button\" id=\"killButton\">KILL</button>");
            client.println("<audio id=\"killSound\" src=\"https://audio.jukehost.co.uk/WZouY1JxlTfazBV9wwZeHlW60jgBsGVa\"></audio>");
            client.println("</div>");
            client.println("<script src=\"https://ajax.googleapis.com/ajax/libs/jquery/3.3.1/jquery.min.js\"></script>");
            client.println("<script>");
            client.println("var slider = document.getElementById(\"escSlider\");");
            client.println("var escP = document.getElementById(\"escPulse\"); escP.innerHTML = slider.value;");
            client.println("slider.oninput = function() { escP.innerHTML = this.value; esc(this.value); };");
            client.println("$.ajaxSetup({timeout:1000}); function esc(pos) { ");
            client.println("$.get(\"/?value=\" + pos + \"&\", {Connection: \"close\"}); }");
            client.println("document.getElementById(\"upButton\").addEventListener(\"click\", function() { adjustESC(25); });");
            client.println("document.getElementById(\"downButton\").addEventListener(\"click\", function() { adjustESC(-25); });");
            client.println("document.getElementById(\"button_25\").addEventListener(\"click\", function() { slider.value = " + button25ValueStr + "; escP.innerHTML = " + button25ValueStr + "; esc(" + button25ValueStr + "); });");
            client.println("document.getElementById(\"button_50\").addEventListener(\"click\", function() { slider.value = " + button50ValueStr + "; escP.innerHTML = " + button50ValueStr + "; esc(" + button50ValueStr + "); });");
            client.println("document.getElementById(\"button_75\").addEventListener(\"click\", function() { slider.value = " + button75ValueStr + "; escP.innerHTML = " + button75ValueStr + "; esc(" + button75ValueStr + "); });");
            client.println("document.getElementById(\"maxButton\").addEventListener(\"click\", function() { slider.value = " + maxSpeedStr + "; escP.innerHTML = " + maxSpeedStr + "; esc(" + maxSpeedStr + "); });");
            client.println("document.getElementById(\"killButton\").addEventListener(\"click\", function() { slider.value = " + minSpeedStr + "; escP.innerHTML = " + minSpeedStr + "; esc(" + minSpeedStr + "); var audio = document.getElementById('killSound'); audio.play(); });");
            client.println("function adjustESC(offset) { var newValue = parseInt(escP.innerHTML) + offset;");
            client.println("if (newValue < " + minSpeedStr + ") { newValue = " + minSpeedStr + "; }");
            client.println("if (newValue > " + maxSpeedStr + ") { newValue = " + maxSpeedStr + "; }");
            client.println("slider.value = newValue; escP.innerHTML = newValue; esc(newValue); }");
            client.println("</script>");
            client.println("</body>");
            client.println("</html>");

            // GET /?value=1240& HTTP/1.1
            if (header.indexOf("GET /?value=") >= 0) {
              pos1 = header.indexOf('=');
              pos2 = header.indexOf('&');
              valueString = header.substring(pos1 + 1, pos2);

              // Set the ESC pulse width
              esc.speed(valueString.toInt());
              // Serial.println(valueString);
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
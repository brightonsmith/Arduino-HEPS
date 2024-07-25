#include <TFT_eSPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_INA228.h>
#include <ArduinoJson.h>
#include <math.h>
#include <SPIFFS.h>
#include <ESP32Servo.h>
#include "ESC.h"

/////////////////////////////////////////////////////////////////////////////Config and Globals////////////////////////////////////////////////////////////////////////////////////////////
// MACROS
#define LED_PIN 2 
#define MIN_SPEED 1140
#define MAX_SPEED 1540
#define ARM_SPEED 1050
#define MIN_ANGLE 30
#define MAX_ANGLE 150

// Pins
#define ONE_WIRE_1 32 // Engine therm
#define ESC_PIN 13
#define SERVO_PIN 25 
#define GPIO_0 0
#define GPIO_35 35

// Function declarations
void IRAM_ATTR isr(); 
void control_pwm();

// WiFi credentials
const char *ssid = "Crazy Frog"; // Set by user
const char *password = "Stravinsky1913"; // Set by user

// Web server
WiFiServer server(80);

// MQTT Broker details
const char *mqtt_broker = "broker.emqx.io";
// const char *mqtt_username = "emqx";
// const char *mqtt_password = "public";
String parent_topic = "weedeater_esp32/";
const int mqtt_port = 1883;
String subscribe_topic = parent_topic + "servo";

WiFiClient espClient;
PubSubClient client(espClient);

TFT_eSPI tft = TFT_eSPI(); // TFT display instance

// Sensors
// INA
Adafruit_INA228 INA1; // Alternator
Adafruit_INA228 INA2; // Battery

// Thermistor
OneWire oneWire1(ONE_WIRE_1);

DallasTemperature thermistor1(&oneWire1);

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

bool readCsvMode = false; // Flag to determine whether to read CSV or sensors
String cmdBuffer = "";

unsigned long startTime; // Global variable to store the starting time

volatile bool isPaused = true; // Interrupt status

// Keep track of timing of recent interrupts
volatile unsigned long button_time = 0;
volatile unsigned long last_button_time = 0;

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
// For internal RAM execution (buton interrupts)
void IRAM_ATTR isr() {
  button_time = millis();
  if (button_time - last_button_time > 250) {
    isPaused = !isPaused;
    last_button_time = button_time;
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TFT_WHITE);
    if (isPaused && !readCsvMode) {
      tft.println("Paused...");
    } else if (!readCsvMode) {
      tft.println("Resuming...");
    }
  }
}

// Setup the display
void setup_display() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("ESP32 initialized!");
  delay(2000);
  tft.fillScreen(TFT_BLACK);

  pinMode(GPIO_0, INPUT_PULLUP); // GPIO 0
  pinMode(GPIO_35, INPUT_PULLUP); // GPIO 35
  attachInterrupt(GPIO_0, isr, FALLING);
  attachInterrupt(GPIO_35, isr, FALLING);
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");

    tft.setCursor(10, 10);
    tft.setTextColor(TFT_RED);
    tft.println("Cannot connect to WiFi.");
    delay(1000);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 10);
  tft.setTextColor(TFT_WHITE);
  tft.println("Connected to WiFi!");
  delay(2000);
  tft.fillScreen(TFT_BLACK);
}

// MQTT callback function
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  
  // Parse JSON
  DynamicJsonDocument doc(256);
  deserializeJson(doc, payload, length);
  
  // Extract servoPos
  int servoPos = doc["servoPos"];

  // Set servo position
  // servo.write(servoPos);

  Serial.print("Set servo position to: ");
  Serial.println(servoPos);
}

void setup_mqtt() {
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) {
      String client_id = "esp32-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
      if (client.connect(client_id.c_str())) {
          Serial.println("Public emqx mqtt broker connected");
          tft.setCursor(10, 10);
          tft.setTextColor(TFT_WHITE);
          tft.println("Connected to MQTT broker!");
          delay(2000);
          tft.fillScreen(TFT_BLACK);
          return;
      } else {
          Serial.print("failed with state ");
          Serial.println(client.state());
          tft.setCursor(10, 10);
          tft.setTextColor(TFT_RED);
          tft.println("Cannot connect to MQTT broker.");
          delay(1000);
          tft.fillScreen(TFT_BLACK);
      }
  }
}

// Reconnect to MQTT
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    control_pwm();
    Serial.print("Reconnecting to MQTT server...");
    // Attempt to connect
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    tft.setCursor(10, 10);
    tft.setTextColor(TFT_RED);
    tft.print("Reconnecting...");

    if (client.connect(client_id.c_str())) {
      Serial.println("connected");
      tft.fillScreen(TFT_BLACK);
    } else {
      Serial.println("failed");
      // Serial.println(" try again in 3 seconds");
      // Wait 3 seconds before retrying
      tft.println("failed.");
      //delay(3000);
      tft.fillScreen(TFT_BLACK);

    }
  }
}

// Setup SPIFFS for data writing
void setup_SPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("Cannot mount SPIFFS.");
    tft.setCursor(10, 10);
    tft.setTextColor(TFT_RED);
    tft.println("Cannout mount SPIFFS.");
    return;
  }

  tft.setCursor(10, 10);
  tft.setTextColor(TFT_WHITE);
  tft.println("SPIFFS mounted!");

  delay(2000);
  tft.fillScreen(TFT_BLACK);
}

// Headers for csv
void write_headers() {
  fs::File INA_file = SPIFFS.open("/INA.csv", FILE_WRITE);
  if(!INA_file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  INA_file.println("time(s),altVoltage(V),batVoltage(V)");
  INA_file.close();

  fs::File therm_file = SPIFFS.open("/therm.csv", FILE_WRITE);
  if(!therm_file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  therm_file.println("time(s),tempBat(F)");
  therm_file.close();
}

// Clear data
void clear_memory(String sensor_type) {
  String filename = "/" + sensor_type + ".csv";
  if (SPIFFS.exists(filename)) {
    Serial.print(filename);
    Serial.println(" exists. Attempting to delete...");
    if (SPIFFS.remove(filename)) {
      Serial.print(filename);
      Serial.println(" successfully deleted.");

      tft.setTextColor(TFT_WHITE);
      tft.print(filename);
      tft.println(" cleared.");
    } else {
      Serial.print("Failed to delete ");
      Serial.println(filename);
    }
  } else {
    Serial.print(filename);
    Serial.println(" does not exist.");
  }
}

// Setup INA sensors
void setup_INA() {
  // INA1 with address 0x40 (A0 = GND, A1 = GND)
  if (!INA1.begin(0x41)) {
      Serial.println("Alternator INA not found, check wiring, address, sensor ID!");
      while (1) delay(10);
  } 
  // INA2 with address 0x41 (A0 = VCC, A1 = GND)
  if (!INA2.begin(0x40)) {
      Serial.println("Battery INA not found, check wiring, address, sensor ID!");
      while (1) delay(10);
  }

  INA1.setShunt(0.1, 1.0);
  INA2.setShunt(0.1, 1.0);

  INA1.setAveragingCount(INA228_COUNT_16);
  INA2.setAveragingCount(INA228_COUNT_16);

  Serial.println("Both INAs found and initialized!");
}

// Setup thermistors
void setup_therm() {
  thermistor1.begin();

  // Change thermistor names as needed
  if (thermistor1.getDeviceCount() == 0) {
    Serial.println("Engine thermistor not found.");
    tft.setTextColor(TFT_RED);
    tft.println("Engine thermistor not found.");
    while (1) delay(10);
  } 

  Serial.println("Thermistor found and initialized!");
  tft.setTextColor(TFT_WHITE);
  tft.println("Thermistor found and initialized!");
}

// Setup servo
void setup_servo() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2000);
  
  int initialPosition = 30; // Example initial position
  servo.write(initialPosition);
  
  tft.setTextColor(TFT_WHITE);
  tft.println("Servo initialized!");
  
  Serial.println("Servo initialized!");
}

// Setup esc
void setup_esc() {
  pinMode(ESC_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // set led to on to indicate arming
  esc.arm(); // Send the Arm command to ESC

  tft.setTextColor(TFT_WHITE);
  tft.println("ESC initialized!");

  Serial.println("ESC initialized!");
}

// Arduino setup function
void setup() {
  Serial.begin(115200);

  setup_display();

  // Check for command to read data from SPIFFS
  if (Serial.available()) {
    delay(2000);
    char cmd = Serial.read();
    if (cmd == 'r') {
      readCsvMode = true; // Set flag to true to indicate read CSV mode
    }
  }

  setup_SPIFFS();

  if (!readCsvMode) {
    Serial.println("Clearing memory.");
    tft.setCursor(10, 10);
    clear_memory("INA");
    clear_memory("therm");

    delay(2000);
    tft.fillScreen(TFT_BLACK);

    write_headers();
    setup_wifi();
    setup_mqtt();

    // Initialize I2C communication
    Wire.begin(21, 22); // SDA = 21, SCL = 22
    tft.setCursor(10, 10);
    // setup_INA();
    setup_therm();

    setup_servo();
    setup_esc();

    delay(2000);
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(10, 10);
    tft.println("Starting in paused mode...");
    tft.println("Press GPIO0 or GPIO35 to start.");
    startTime = millis();
  } else {
    tft.setCursor(10, 10);
    tft.setTextColor(TFT_WHITE);
    tft.println("Starting in CSV mode.");
  }
}

////////////////////////////////////////////////////////////////////////////////////////Loop/////////////////////////////////////////////////////////////////////////////////////////////////
// Read INA sensors
void read_INA() {

  float busVoltage1 = (INA1.readBusVoltage() / 1000.0); 
  float busVoltage2 = (INA2.readBusVoltage() / 1000.0);
  Serial.print("Alternator voltage: ");
  Serial.print(busVoltage1);
  Serial.println(" V");
  Serial.print("Battery voltage: ");
  Serial.print(busVoltage2);
  Serial.println(" V");

  // Manually construct the JSON payload
  char payload[512];
  snprintf(payload, sizeof(payload),
    "{\"voltageAlt\":%f,\"voltageBat\":%f}",
    busVoltage1, busVoltage2);

  //Serial.println(payload); // Print the payload to Serial for debugging
  String INA_topic = parent_topic + "INA";

  client.publish(INA_topic.c_str(), payload);

  //delay(100);

  // Calculate the elapsed time in seconds
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000.0; // Convert milliseconds to seconds

  // Save data to CSV file
  fs::File file = SPIFFS.open("/INA.csv", FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  file.printf("%f,%f\n", 
              elapsedTime, busVoltage1, busVoltage2);
  file.close();
}

// Read thermistors
void read_therm() {
  tft.print("Reading Therm data...");

  thermistor1.requestTemperatures();
  control_pwm();

  float temp_1 = thermistor1.getTempFByIndex(0);

  Serial.print("Engine temp: ");
  Serial.print(temp_1);
  Serial.println(" F");

  // Manually construct the JSON payload
  char payload[256];
  snprintf(payload, sizeof(payload),
    "{\"tempEngine\":%f}",
    temp_1);

  //Serial.println(payload); // Print the payload to Serial
  String therm_topic = parent_topic + "therm";

  client.publish(therm_topic.c_str(), payload);

  if (!isPaused) {
    tft.println("Published!");
  }

  delay(10);

  // Calculate the elapsed time in seconds
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000.0; // Convert milliseconds to seconds

  // Save data to CSV file
  fs::File file = SPIFFS.open("/therm.csv", FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  file.printf("%f,%f\n", 
              elapsedTime, temp_1);
  file.close();
}

// Save data locally
void read_csv(String sensor_type) {
  String filename = "/" + sensor_type + ".csv";
  fs::File file = SPIFFS.open(filename, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
  Serial.println("EOF");
}

String read_sensors() {
  // INA
  // float busVoltage1 = INA1.readBusVoltage() / 1000.0;
  // float busVoltage2 = INA2.readBusVoltage() / 1000.0;

  // Thermistors
  thermistor1.requestTemperatures();
  float temp_1 = thermistor1.getTempFByIndex(0);

  // Convert the sensor readings to JSON format
  String json = "{\"voltageAlt\":" + String(0.0) + ",\"voltageBat\":" + String(0.0) + ",\"tempEngine\":" + String(temp_1) + "}";
  return json;
}

// ESC and Servo control
void control_pwm() {
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
            if (header.indexOf("GET /sensorData") >= 0 ) {
                String data = read_sensors(); // "{\"voltageAlt\":0.0,\"voltageBat\":0.0,\"tempEngine\":0.0}";
                client.println("HTTP/1.1 200 OK");
                client.println("Content-type:application/json");
                client.println("Connection: close");
                client.println();
                client.print(data);
                break;
              } else {
                client.println("HTTP/1.1 200 OK");
                client.println("Content-type:text/html");
                client.println();

                // HTML content
                client.println("<!DOCTYPE html><html lang=\"en\"><head>");
                client.println("<meta charset=\"UTF-8\">");
                client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=0.8\">");
                client.println("<style>");
                client.println("body { display: flex; justify-content: center; align-items: center; height: 100vh; font-family: Arial, sans-serif; margin: 0; padding: 0; box-sizing: border-box}");
                client.println(".container-wrapper { width: 100%; max-width: 600px; text-align: center; }");
                client.println(".container { margin-bottom: 20px; box-sizing: border-box; }");
                client.println(".slider { width: 100%; max-width: 300px; }");
                client.println(".button { display: inline-block; padding: 10px 20px; font-size: 20px; cursor: pointer; text-align: center; text-decoration: none; outline: none; color: #fff; background-color: #4CAF50; border: none; border-radius: 15px; box-shadow: 0 9px #999; }");
                client.println(".button:hover { background-color: #3e8e41; }");
                client.println(".button:active { background-color: #3e8e41; box-shadow: 0 5px #666; transform: translateY(4px); }");
                client.println("#escKillButton, #servoKillButton { background-color: red; color: white; padding: 20px 40px; font-size: 24px; margin-top: 20px; }");
                client.println("hr { border: 1px solid #ccc; margin: 20px 0; }");
                client.println("@media screen and (orientation: portrait) { .container-wrapper { max-width: 600px; flex-direction: column; } .container { width: 100%; margin-bottom: 20px; } .bottom-section { display: block; } }");
                client.println("@media screen and (orientation: landscape) { .container-wrapper { display: flex; flex-direction: row; justify-content: space-between; align-items: flex-start; max-width: 1200px; flex-wrap: wrap; } .container { width: 45%; margin-bottom: 0; box-sizing: border-box; } .bottom-section { width: 100%; display: flex; justify-content: space-between; align-items: center; flex-wrap: wrap; margin-top: 20px; } hr { display: none; } .container-wrapper::before { content: \"\"; display: block; width: 1px; height: 100%; background-color: #ccc; } }");
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

                client.println("<div class=\"bottom-section\">");
                client.println("<div class=\"container\">");
                client.println("<p>Alternator Voltage: <span id=\"alternatorVoltage\"></span> V </p>");
                client.println("<p>Battery Voltage: <span id=\"batteryVoltage\"></span> V </p>");
                client.println("<p>Engine Temperature: <span id=\"engineTemp\"></span>  &#176;F</p>");
                client.println("</div>");
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

                // Function to fetch and display sensor data
                client.println("function fetchSensorData() {");
                client.println("$.get('/sensorData', function(data) {");
                client.println("$('#alternatorVoltage').text(data.voltageAlt);");
                client.println("$('#batteryVoltage').text(data.voltageBat);");
                client.println("$('#engineTemp').text(data.tempEngine);");
                client.println("}, 'json');");
                client.println("}");
                client.println("setInterval(fetchSensorData, 1000);"); // Fetch data every second

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
              }
    
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

void loop() {
  control_pwm();

  // Check for command to read data from SPIFFS
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'r') {
      readCsvMode = true; // Set flag to true to indicate read CSV mode
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(10, 10);
      tft.setTextColor(TFT_WHITE);
      tft.println("Entered CSV mode.");

      cmdBuffer = ""; // Clear buffer
    } else if (readCsvMode) {
        cmdBuffer += cmd;
        switch (cmdBuffer.charAt(0)) {
          case 'I':
            read_csv("IMU");
            cmdBuffer = "";
            break;
          case 'B':
            read_csv("BME");
            cmdBuffer = "";
            break;
          case 'V':
            read_csv("INA");
            cmdBuffer = "";
            break;
          case 'H':
            read_csv("hall");
            cmdBuffer = "";
            break;
          case 'T':
            read_csv("therm");
            cmdBuffer = "";
            break;
          default:
            Serial.println("Invalid sensor type.");
            readCsvMode = false;
            cmdBuffer = "";
            break;
        }
    }
  }    
  // Read and publish data if not in read CSV mode
  if (!readCsvMode) {
    if (!isPaused) {
      tft.fillScreen(TFT_BLACK); // Clear screen
      if (!client.connected()) {
        control_pwm();
        reconnect();
        control_pwm();
    }
    client.loop();
    tft.setCursor(10, 10);
    tft.setTextColor(TFT_WHITE);

    // read_INA();
    read_therm();
    }
  }
}

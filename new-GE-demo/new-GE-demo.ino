#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_INA228.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <math.h>
#include <SPIFFS.h>
#include <ESP32Servo.h>

///////////////////////////////////////////////////////////////////////////// Config and Globals ////////////////////////////////////////////////////////////////////////////////////////////
// Pins
#define ONE_WIRE_1 32 // Battery thermistor
#define SERVO_PIN 25
#define GPIO_0 0

// WiFi credentials
const char *ssid = "Crazy Frog"; // Set by user
const char *password = "Stravinsky1913"; // Set by user

// MQTT Broker details
const char *mqtt_broker = "broker.emqx.io";
String parent_topic = "weedeater_esp32/";
const int mqtt_port = 1883;
String subscribe_topic = parent_topic + "servo";

WiFiClient espClient;
PubSubClient client(espClient);

// Sensors
// INA
Adafruit_INA228 INA1; // Alternator
Adafruit_INA228 INA2; // Battery

// Thermistor
OneWire oneWire1(ONE_WIRE_1);

DallasTemperature thermistor1(&oneWire1);

// Servo
Servo servo; // Servo instance

bool readCsvMode = false; // Flag to determine whether to read CSV or sensors
String cmdBuffer = "";

unsigned long startTime; // Global variable to store the starting time

volatile bool isPaused = false; // Interrupt status

// Keep track of timing of recent interrupts
volatile unsigned long button_time = 0;
volatile unsigned long last_button_time = 0;

///////////////////////////////////////////////////////////////////////////////////////// Setup ////////////////////////////////////////////////////////////////////////////////////////////
// For internal RAM execution
void IRAM_ATTR isr() {
  button_time = millis();
  if (button_time - last_button_time > 250) {
    isPaused = !isPaused;
    last_button_time = button_time;
  }
}

// Setup the display
void setup_display() {
  pinMode(GPIO_0, INPUT_PULLUP); // GPIO 0
  attachInterrupt(GPIO_0, isr, FALLING);
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
  servo.write(servoPos);

  Serial.print("Set servo position to: ");
  Serial.println(servoPos);
}

// Setup WiFi connection
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Setup MQTT connection
void setup_mqtt() {
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
    
    if (client.connect(client_id.c_str())) {
      Serial.println("Public emqx mqtt broker connected");

      // Subscribe to the topic
      client.subscribe(subscribe_topic.c_str());
      Serial.printf("Subscribed to topic: %s\n", subscribe_topic.c_str());
      return;
    } else {
      Serial.print("failed with state ");
      Serial.println(client.state());
    }
  }
}

// Reconnect to MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Reconnecting to MQTT server...");
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());

    if (client.connect(client_id.c_str())) {
      Serial.println("connected");

      // Subscribe to the topic
      client.subscribe(subscribe_topic.c_str());
      Serial.printf("Subscribed to topic: %s\n", subscribe_topic.c_str());

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 3 seconds");
    }
  }
}

// Setup SPIFFS for data writing
void setup_SPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("Cannot mount SPIFFS.");
    return;
  }
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
    Serial.println("Battery thermistor not found.");
    while (1) delay(10);
  } 

  Serial.println("Thermistors found and initialized!");
}

// Setup servo
void setup_servo() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2000);
  
  int initialPosition = 180; // Example initial position
  servo.write(initialPosition);
  
  Serial.println("Servo initialized!");
}

// Arduino setup function
void setup() {
  Serial.begin(9600);

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
    clear_memory("INA");
    clear_memory("therm");

    delay(2000);

    write_headers();
    setup_wifi();
    setup_mqtt();

    // Initialize I2C communication
    Wire.begin(21, 22); // SDA = 21, SCL = 22
    setup_INA();
    setup_therm();
    setup_servo();

    delay(1000);
    startTime = millis();
  }
}

////////////////////////////////////////////////////////////////////////////////////////Loop/////////////////////////////////////////////////////////////////////////////////////////////////
// Read INA sensors
void read_INA() {

  float busVoltage1 = (INA1.readBusVoltage() / 1000.0) + 0.39; 
  float busVoltage2 = (INA2.readBusVoltage() / 1000.0) + 0.24;
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

  thermistor1.requestTemperatures();

  float temp_1 = thermistor1.getTempFByIndex(0);

  Serial.print("Battery temp: ");
  Serial.print(temp_1);
  Serial.println(" F");

  // Manually construct the JSON payload
  char payload[256];
  snprintf(payload, sizeof(payload),
    "{\"tempBat\":%f}",
    temp_1);

  //Serial.println(payload); // Print the payload to Serial
  String therm_topic = parent_topic + "therm";

  client.publish(therm_topic.c_str(), payload);

  //delay(10);

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

// // Read voltages
// void read_voltages() {
//   // Subtract the last reading
//   totalBat = totalBat - readingsBat[readIndex];
//   totalAlt = totalAlt - readingsAlt[readIndex];

//   // Read new values
//   int analogBat = analogRead(BATTERY_VOLTAGE_PIN);
//   int analogAlt = analogRead(ALTERNATOR_VOLTAGE_PIN);

//   // Calculate actual voltages
//   float voltageBat = (analogBat * (3.3 / 4095.0) * (R1_bat + R2_bat) / R2_bat) + 2;
//   float voltageAlt = (analogAlt * (3.3 / 4095.0) * (R1_alt + R2_alt) / R2_alt) + 2;

//   // Add the new reading
//   readingsBat[readIndex] = voltageBat;
//   readingsAlt[readIndex] = voltageAlt;
//   totalBat = totalBat + readingsBat[readIndex];
//   totalAlt = totalAlt + readingsAlt[readIndex];

//   // Advance to the next position in the array
//   readIndex = readIndex + 1;
//   if (readIndex >= numReadings) {
//     readIndex = 0;
//   }

//   // Calculate the average
//   averageBat = totalBat / numReadings;
//   averageAlt = totalAlt / numReadings;

//   // Print smoothed voltages
//   Serial.print("Battery voltage (smoothed): ");
//   Serial.print(averageBat);
//   Serial.println(" V");
//   Serial.print("Alternator voltage (smoothed): ");
//   Serial.print(averageAlt);
//   Serial.println(" V");

//   // Manually construct the JSON payload
//   char payload[512];
//   snprintf(payload, sizeof(payload),
//     "{\"voltageAlt\":%f,\"voltageBat\":%f}",
//     averageAlt, averageBat);

//   //Serial.println(payload); // Print the payload to Serial for debugging
//   String INA_topic = parent_topic + "INA";

//   client.publish(INA_topic.c_str(), payload);

//   //delay(100);

//   // Calculate the elapsed time in seconds
//   unsigned long currentTime = millis();
//   float elapsedTime = (currentTime - startTime) / 1000.0; // Convert milliseconds to seconds

//   // Save data to CSV file
//   fs::File file = SPIFFS.open("/INA.csv", FILE_APPEND);
//   if(!file) {
//     Serial.println("Failed to open file for appending");
//     return;
//   }
//   file.printf("%f,%f\n", 
//               elapsedTime, averageAlt, averageBat);
//   file.close();
// }

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

void loop() {
  // Check for command to read data from SPIFFS
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'r') {
      readCsvMode = true; // Set flag to true to indicate read CSV mode

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
      if (!client.connected()) {
      reconnect();
    }
    client.loop();

    read_INA();
    read_therm();

    delay(1000);
    }
  }
}
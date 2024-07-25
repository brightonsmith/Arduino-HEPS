// #include <TFT_eSPI.h>
#include <Wire.h>
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
#define ONE_WIRE_1 32 // Battery therm
#define SERVO_PIN 25 
#define GPIO_0 0
#define GPIO_35 35

// Function declarations
void IRAM_ATTR isr();

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

// TFT_eSPI tft = TFT_eSPI(); // TFT display instance

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

volatile bool isPaused = true; // Interrupt status

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
    // tft.fillScreen(TFT_BLACK);
    // tft.setCursor(10, 10);
    // tft.setTextColor(TFT_WHITE);
    // if (isPaused && !readCsvMode) {
    //   tft.println("Paused...");
    // } else if (!readCsvMode) {
    //   tft.println("Resuming...");
    // }
  }
}

// Setup the display
void setup_display() {
  // tft.init();
  // tft.setRotation(1);
  // tft.fillScreen(TFT_BLACK);
  // tft.setTextColor(TFT_WHITE, TFT_BLACK);
  // tft.setTextSize(2);
  // tft.setCursor(10, 10);
  // tft.println("ESP32 initialized!");
  // delay(2000);
  // tft.fillScreen(TFT_BLACK);

  pinMode(GPIO_0, INPUT_PULLUP); // GPIO 0
  pinMode(GPIO_35, INPUT_PULLUP); // GPIO 35
  attachInterrupt(GPIO_0, isr, FALLING);
  attachInterrupt(GPIO_35, isr, FALLING);
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

    // tft.setCursor(10, 10);
    // tft.setTextColor(TFT_RED);
    // tft.println("Cannot connect to WiFi.");
    delay(1000);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // tft.fillScreen(TFT_BLACK);
  // tft.setCursor(10, 10);
  // tft.setTextColor(TFT_WHITE);
  // tft.println("Connected to WiFi!");
  // delay(2000);
  // tft.fillScreen(TFT_BLACK);
}

// Setup MQTT connection
void setup_mqtt() {
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    
    if (client.connect(client_id.c_str())) {
      Serial.println("Public emqx mqtt broker connected");
      // tft.setCursor(10, 10);
      // tft.setTextColor(TFT_WHITE);
      // tft.println("Connected to MQTT broker!");
      // delay(2000);
      // tft.fillScreen(TFT_BLACK);

      // Subscribe to the topic
      client.subscribe(subscribe_topic.c_str());
      Serial.printf("Subscribed to topic: %s\n", subscribe_topic.c_str());
      // tft.setCursor(10, 10);
      // tft.setTextColor(TFT_WHITE);
      // tft.println("Subscribed to topic!");
      // delay(2000);
      // tft.fillScreen(TFT_BLACK);
      return;
    } else {
      Serial.print("failed with state ");
      Serial.println(client.state());
      // tft.setCursor(10, 10);
      // tft.setTextColor(TFT_RED);
      // tft.println("Cannot connect to MQTT broker.");
      // delay(1000);
      // tft.fillScreen(TFT_BLACK);
    }
  }
}

// Reconnect to MQTT
void reconnect() {
  while (!client.connected()) {
    Serial.print("Reconnecting to MQTT server...");
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    // tft.setCursor(10, 10);
    // tft.setTextColor(TFT_RED);
    // tft.print("Reconnecting...");

    if (client.connect(client_id.c_str())) {
      Serial.println("connected");
      // tft.fillScreen(TFT_BLACK);

      // Subscribe to the topic
      client.subscribe(subscribe_topic.c_str());
      Serial.printf("Subscribed to topic: %s\n", subscribe_topic.c_str());
      // tft.setCursor(10, 10);
      // tft.setTextColor(TFT_WHITE);
      // tft.println("Subscribed to topic!");
      // delay(2000);
      // tft.fillScreen(TFT_BLACK);

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 3 seconds");
      // tft.println("failed.");
      delay(3000);
      // tft.fillScreen(TFT_BLACK);
    }
  }
}

// Setup SPIFFS for data writing
void setup_SPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("Cannot mount SPIFFS.");
    // tft.setCursor(10, 10);
    // tft.setTextColor(TFT_RED);
    // tft.println("Cannout mount SPIFFS.");
    return;
  }

  // tft.setCursor(10, 10);
  // tft.setTextColor(TFT_WHITE);
  // tft.println("SPIFFS mounted!");

  // delay(2000);
  // tft.fillScreen(TFT_BLACK);
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

      // tft.setTextColor(TFT_WHITE);
      // tft.print(filename);
      // tft.println(" cleared.");
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
    // tft.setTextColor(TFT_RED);
    // tft.println("Battery thermistor not found.");
    while (1) delay(10);
  } 

  Serial.println("Battery thermistor found and initialized!");
  // tft.setTextColor(TFT_WHITE);
  // tft.println("Battery thermistor found and initialized!");
}

// Setup servo
void setup_servo() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2000);
  
  int initialPosition = 0; // Example initial position
  servo.write(initialPosition);
  
  // tft.setTextColor(TFT_WHITE);
  // tft.println("Servo initialized!");
  
  Serial.println("Servo initialized!");
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
    // tft.setCursor(10, 10);
    clear_memory("INA");
    clear_memory("therm");

    delay(2000);
    // tft.fillScreen(TFT_BLACK);

    write_headers();
    setup_wifi();
    setup_mqtt();

    // Initialize I2C communication
    Wire.begin(21, 22); // SDA = 21, SCL = 22
    // tft.setCursor(10, 10);
    setup_INA();
    setup_therm();
    setup_servo();
    delay(1000);
    // tft.fillScreen(TFT_BLACK);
    // tft.setCursor(10, 10);
    // tft.println("Starting in paused mode...");
    // tft.println("Press GPIO0 or GPIO35 to start.");
    startTime = millis();
  } // else {
  //   tft.setCursor(10, 10);
  //   tft.setTextColor(TFT_WHITE);
  //   tft.println("Starting in CSV mode.");
  // }
}

////////////////////////////////////////////////////////////////////////////////////////Loop/////////////////////////////////////////////////////////////////////////////////////////////////
// Read INA sensors
void read_INA() {
  // tft.print("Reading INA data...");

  float busVoltage1 = INA1.readBusVoltage() / 1000.0; 
  float busVoltage2 = INA2.readBusVoltage() / 1000.0;
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

  // tft.println("Published!");

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
  // tft.print("Reading Therm data...");

  thermistor1.requestTemperatures();

  float temp_1 = thermistor1.getTempFByIndex(0);

  Serial.print("Battery temp: ");
  Serial.print(temp_1);
  Serial.println(" F");

  // Manually construct the JSON payload
  char payload[256];
  snprintf(payload, sizeof(payload),
    "{\"tempBattery\":%f}",
    temp_1);

  //Serial.println(payload); // Print the payload to Serial
  String therm_topic = parent_topic + "therm";

  client.publish(therm_topic.c_str(), payload);

  // if (!isPaused) {
  //   tft.println("Published!");
  // }

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

void loop() {
  // Check for command to read data from SPIFFS
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'r') {
      readCsvMode = true; // Set flag to true to indicate read CSV mode
      // tft.fillScreen(TFT_BLACK);
      // tft.setCursor(10, 10);
      // tft.setTextColor(TFT_WHITE);
      // tft.println("Entered CSV mode.");

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
      // tft.fillScreen(TFT_BLACK); // Clear screen
      if (!client.connected()) {
      reconnect();
    }
    client.loop();

    // tft.setCursor(10, 10);
    // tft.setTextColor(TFT_WHITE);

    read_INA();
    read_therm();

    // tft.fillScreen(TFT_BLACK);
    }
  }
}
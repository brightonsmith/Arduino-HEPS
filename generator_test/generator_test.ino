#include <TFT_eSPI.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <MPU9250.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_INA260.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <math.h>
#include <SPIFFS.h>
#include <ESP32Servo.h>

/////////////////////////////////////////////////////////////////////////////Config and Globals////////////////////////////////////////////////////////////////////////////////////////////
// MACROS
#define C2F(c) ((c) * 1.8 + 32)
#define RAD2DEG(r) ((r) * 180.0 / M_PI)
#define SEALEVELPRESSURE_HPA 1013.25
#define ADC_MAX 4095.0 // 12-bit ADC for ESP32
#define VREF 3.3 // Reference voltage for ESP32 ADC
#define VOLTAGE_SCALE (69.0 / 3.3) // 69V max voltage corresponding to 3.3V ADC input
#define CURRENT_SENSITIVITY 0.02 // 20mV/A

// Pins
#define ONE_WIRE_1 25 // Engine therm
#define ONE_WIRE_2 26 // Resistor 1 therm
#define ONE_WIRE_3 27 // Resistor 2 therm
#define SERVO_PIN 13 
#define GPIO_0 0
#define GPIO_35 35

// Function declarations
void IRAM_ATTR isr(); 
void setup_display();
void setup_wifi();
void setup_mqtt();
void reconnect();
void callback(char *topic, byte *payload, unsigned int length);
void setup_SPIFFS();
void write_headers();
void clear_memory();
void setup_IMU();
void setup_BME();
void setup_INA();
void setup_therm();
void setup_servo();
void read_IMU();
void read_BME();
void read_INA();
void real_hall();
void read_therm();
String read_sensors();
void control_servo();
void read_csv();

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

WiFiClient espClient;
PubSubClient client(espClient);

TFT_eSPI tft = TFT_eSPI(); // TFT display instance

// Sensors
// IMU
MPU9250 IMU(i2c0, 0x68);

// BME
Adafruit_BME280 BME;

// Thermistors
OneWire oneWire1(ONE_WIRE_1);
OneWire oneWire2(ONE_WIRE_2);
OneWire oneWire3(ONE_WIRE_3);

DallasTemperature thermistor1(&oneWire1);
DallasTemperature thermistor2(&oneWire2);
DallasTemperature thermistor3(&oneWire3);

// Hall Effect
const int voltagePinAlternator = 12;
const int currentPinAlternator = 13;

// INA
Adafruit_INA260 INA1; // Alternator
Adafruit_INA260 INA2; // Battery

// Servo
Servo servo;

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
String valueString = String(5);
int pos1 = 0;
int pos2 = 0;

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

void callback(char *topic, byte *payload, unsigned int length) {
    control_servo();
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message: ");

    for (int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    Serial.println();
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

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    control_servo();
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
  fs::File IMU_file = SPIFFS.open("/IMU.csv", FILE_WRITE);
  if(!IMU_file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  IMU_file.println("time(s),accelX(m/s^2),accelY(m/s^2),accelZ(m/s^2),accelMagnitude(m/s^2),gyroX(deg/s),gyroY(deg/s),gyroZ(deg/s),magX(uT),magY(uT),magZ(uT),magMagnitude(uT)");
  IMU_file.close();

  fs::File BME_file = SPIFFS.open("/BME.csv", FILE_WRITE);
  if(!BME_file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  BME_file.println("time(s),temp(F),humidity(%RH),pressure(bar),altitude(m)");
  BME_file.close();

  fs::File INA_file = SPIFFS.open("/INA.csv", FILE_WRITE);
  if(!INA_file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  INA_file.println("time(s),altVoltage(V),batVoltage(V)");
  INA_file.close();

  fs::File hall_file = SPIFFS.open("/hall.csv", FILE_WRITE);
  if(!hall_file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  hall_file.println("time(s),altVoltage(V),altCurrent(A),altPower(W),altADCVoltage(nan),altADCCurrent(nan)");
  hall_file.close();

  fs::File therm_file = SPIFFS.open("/therm.csv", FILE_WRITE);
  if(!therm_file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  therm_file.println("time(s),tempEngine(F),tempBattery(F),tempResistor(F)");
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
      
// Setup IMU sensor
void setup_IMU() {
  // Initialize
  if (!IMU.begin()) {
    Serial.println("Could not find a valid MPU9250 sensor, check wiring, address, sensor ID!");
    tft.setTextColor(TFT_RED);
    tft.println("IMU not found.");
    while (1) delay(10);
  }

  Serial.println("IMU found and initialized!");
  tft.setTextColor(TFT_WHITE);
  tft.println("IMU found and initialized!");

  // Configure
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);

  // Calibrate
  IMU.calibrateAccel();
  IMU.setAccelCalX(0.55, 1.00);
  IMU.setAccelCalY(-0.4, 1.00);
  IMU.setAccelCalZ(-8.85, 1.00); 

  IMU.calibrateGyro();
  IMU.calibrateMag();
}

// Setup BME sensor
void setup_BME() {
  unsigned status;
    
  // default settings
  status = BME.begin();  
  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(BME.sensorID(),16);
      tft.setTextColor(TFT_RED);
      tft.println("BME not found.");
      while (1) delay(10);
  }

  Serial.println("BME found and initialized!");
  tft.setTextColor(TFT_WHITE);
  tft.println("BME found and initialized");
}

// Setup INA sensors
void setup_INA() {
  // INA1 with address 0x40 (A0 = GND, A1 = GND)
  if (!INA1.begin(0x40)) {
      Serial.println("Alternator INA not found, check wiring, address, sensor ID!");
      tft.setTextColor(TFT_RED);
      tft.println("Alternator INA not found.");
      while (1) delay(10);
  } 
  // INA2 with address 0x41 (A0 = VCC, A1 = GND)
  if (!INA2.begin(0x41)) {
      Serial.println("Battery INA not found, check wiring, address, sensor ID!");
      tft.setTextColor(TFT_RED);
      tft.println("Battery INA not found.");
      while (1) delay(10);
  }
  Serial.println("Both INAs found and initialized!");
  tft.setTextColor(TFT_WHITE);
  tft.println("Both INAs found and initialized");
}

// Setup thermistors
void setup_therm() {
  thermistor1.begin();
  thermistor2.begin();
  thermistor3.begin();

  // Change thermistor names as needed
  if (thermistor1.getDeviceCount() == 0) {
    Serial.println("Engine thermistor not found.");
    tft.setTextColor(TFT_RED);
    tft.println("Engine thermistor not found.");
    while (1) delay(10);
  } 

  if (thermistor2.getDeviceCount() == 0) {
    Serial.println("Resistor 1 thermistor not found.");
    tft.setTextColor(TFT_RED);
    tft.println("Resistor 1 thermistor not found.");
    while (1) delay(10);
  } 
  if (thermistor3.getDeviceCount() == 0) {
    Serial.println("Resistor 2 thermistor not found.");
    tft.setTextColor(TFT_RED);
    tft.println("Resistor 2 thermistor not found.");
    while (1) delay(10); 
  }

  Serial.println("All thermistors found and initialized!");
  tft.setTextColor(TFT_WHITE);
  tft.println("All thermistors found and initialized!");
}

// Setup servo
void setup_servo() {
  tft.fillScreen(TFT_BLACK);
  servo.attach(SERVO_PIN);
  
  // Sweep from 0 to 180 degrees
  for (int pos = 0; pos <= 180; pos++) {
    servo.write(pos);
    delay(5); // Adjust delay as needed 
  }

  // Sweep back from 180 to 0 degrees
  for (int pos = 180; pos >= 0; pos--) {
    servo.write(pos);
    delay(5); // Adjust delay as needed 
  }

  tft.setCursor(10, 10);
  Serial.println("Servo initialized!");
  
  tft.setTextColor(TFT_WHITE);
  tft.println("Servo initialized!");
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
    clear_memory("IMU");
    clear_memory("BME");
    clear_memory("INA");
    clear_memory("hall");
    clear_memory("therm");

    delay(2000);
    tft.fillScreen(TFT_BLACK);

    write_headers();
    setup_wifi();
    setup_mqtt();

    // Initialize I2C communication
    Wire.begin(21, 22); // SDA = 21, SCL = 22
    tft.setCursor(10, 10);
    setup_IMU(); // Edit
    setup_BME(); // Edit
    setup_INA(); // Edit
    setup_therm(); // Edit
    setup_servo();
    delay(1000);
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
// Function to read and publish IMU data
void read_IMU() {
  tft.print("Reading IMU data...");
  IMU.readSensor();

  // Get acceleration components (m/s^2)
  float accelX = IMU.getAccelX_mss();
  float accelY = IMU.getAccelY_mss();
  float accelZ = IMU.getAccelZ_mss();
  float accelMagnitude = sqrt(sq(accelX) + sq(accelY) + sq(accelZ));

  // Get gyroscope components (deg)
  float gyroX = RAD2DEG(IMU.getGyroX_rads());
  float gyroY = RAD2DEG(IMU.getGyroY_rads());
  float gyroZ = RAD2DEG(IMU.getGyroZ_rads());

  // Get magnetic field components (uT)
  float magX = IMU.getMagX_uT();
  float magY = IMU.getMagY_uT();
  float magZ = IMU.getMagZ_uT();
  float magMagnitude = sqrt(sq(magX) + sq(magY) + sq(magZ));

  // Manually construct the JSON payload
  char payload[512];
  snprintf(payload, sizeof(payload),
    "{\"accelX\":%f,\"accelY\":%f,\"accelZ\":%f,\"accelMagnitude\":%f,\"gyroX\":%f,\"gyroY\":%f,\"gyroZ\":%f,\"magX\":%f,\"magY\":%f,\"magZ\":%f,\"magMagnitude\":%f}",
    accelX, accelY, accelZ, accelMagnitude, gyroX, gyroY, gyroZ, magX, magY, magZ, magMagnitude);

  //Serial.println(payload); // Print the payload to Serial
  String IMU_topic = parent_topic + "IMU";

  client.publish(IMU_topic.c_str(), payload);

  tft.println("Published!");

  delay(10);

  // Calculate the elapsed time in seconds
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000.0; // Convert milliseconds to seconds

  // Save data to CSV file
  fs::File file = SPIFFS.open("/IMU.csv", FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  file.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", 
              elapsedTime, accelX, accelY, accelZ, accelMagnitude,
              gyroX, gyroY, gyroZ,
              magX, magY, magZ, magMagnitude);
  file.close();
}

void read_BME() {
  tft.print("Reading BME data...");

  float temp = C2F(BME.readTemperature()); // Convert to Fahrenheit
  float humidity = BME.readHumidity(); // %RH
  float pressure = BME.readPressure() * 0.00001; // Convert to bar
  float altitude = BME.readAltitude(SEALEVELPRESSURE_HPA); 

  // Serial.print("Temperature = ");
  // Serial.print(BME.readTemperature());
  // Serial.println(" °C");

  // Serial.print("Pressure = ");

  // Serial.print(BME.readPressure() / 100.0F);
  // Serial.println(" hPa");

  // Serial.print("Approx. Altitude = ");
  // Serial.print(BME.readAltitude(SEALEVELPRESSURE_HPA));
  // Serial.println(" m");

  // Serial.print("Humidity = ");
  // Serial.print(BME.readHumidity());
  // Serial.println(" %");

  // Manually construct the JSON payload
  char payload[512];
  snprintf(payload, sizeof(payload),
    "{\"temp\":%f,\"humidity\":%f,\"pressure\":%f,\"altitude\":%f}",
    temp, humidity, pressure, altitude);

  //Serial.println(payload); // Print the payload to Serial for debugging
  String BME_topic = parent_topic + "BME";

  client.publish(BME_topic.c_str(), payload);

  tft.println("Published!");

  delay(10);

  // Calculate the elapsed time in seconds
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000.0; // Convert milliseconds to seconds

  // Save data to CSV file
  fs::File file = SPIFFS.open("/BME.csv", FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  file.printf("%f,%f,%f,%f,%f\n", 
              elapsedTime, temp, humidity, pressure, altitude);
  file.close();
}

// Read INA sensors
void read_INA() {
  tft.print("Reading INA data...");

  float busVoltage1 = INA1.readBusVoltage() / 1000.0; 
  float busVoltage2 = INA2.readBusVoltage() / 1000.0;

  // Manually construct the JSON payload
  char payload[512];
  snprintf(payload, sizeof(payload),
    "{\"voltageAlt\":%f,\"voltageBat\":%f}",
    busVoltage1, busVoltage2);

  //Serial.println(payload); // Print the payload to Serial for debugging
  String INA_topic = parent_topic + "INA";

  client.publish(INA_topic.c_str(), payload);

  tft.println("Published!");

  delay(10);

  // Calculate the elapsed time in seconds
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000.0; // Convert milliseconds to seconds

  // Save data to CSV file
  fs::File file = SPIFFS.open("/INA.csv", FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  file.printf("%f,%f,%f\n", 
              elapsedTime, busVoltage1, busVoltage2);
  file.close();
}

// Read Hall effect sensors
void read_hall() {
  tft.print("Reading Hall data...");
  
  float voltageADCAlternator = analogRead(voltagePinAlternator);
  float currentADCAlternator = analogRead(currentPinAlternator);

  // float voltage_alternator = map(voltageADCAlternator, 0, 4095, 0.0f, 69.0f);
  // float current_alternator = map(currentADCAlternator, 0, 4095, 0.0f, 150.0f);
  float voltage_alternator = (voltageADCAlternator / ADC_MAX) * VREF * VOLTAGE_SCALE;
  float current_alternator = (currentADCAlternator / ADC_MAX) * VREF / CURRENT_SENSITIVITY;
  float power_alternator = voltage_alternator * current_alternator;

  // Manually construct the JSON payload
  char payload[256];
  snprintf(payload, sizeof(payload),
    "{\"voltageAlt\":%f,\"currentAlt\":%f,\"powerAlt\":%f}",
    voltage_alternator, current_alternator, power_alternator);

  //Serial.println(payload); // Print the payload to Serial
  String hall_topic = parent_topic + "hall";

  client.publish(hall_topic.c_str(), payload);

  tft.println("Published!");

  delay(10);

  // Calculate the elapsed time in seconds
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000.0; // Convert milliseconds to seconds

  // Save data to CSV file
  fs::File file = SPIFFS.open("/hall.csv", FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  file.printf("%f,%f,%f,%f,%f,%f\n", 
              elapsedTime, voltage_alternator, current_alternator, power_alternator, voltageADCAlternator, currentADCAlternator);
  file.close();
}

void read_therm() {
  tft.print("Reading Therm data...");

  unsigned long test_start_time = millis();
  thermistor1.requestTemperatures();
  control_servo();
  unsigned long end_time1 = millis();
  thermistor2.requestTemperatures();
  control_servo();
  unsigned long end_time2 = millis();
  thermistor3.requestTemperatures();
  control_servo();
  unsigned long end_time3 = millis();

  float temp_1 = thermistor1.getTempFByIndex(0);
  float temp_2 = thermistor2.getTempFByIndex(0);
  float temp_3 = thermistor3.getTempFByIndex(0);

  unsigned long test_end_time = millis();

  // Serial.print("Elapsed time for thermistor1: ");
  // Serial.print(end_time1 - test_start_time);
  // Serial.println(" ms");

  // Serial.print("Elapsed time for thermistor2: ");
  // Serial.print(end_time2 - end_time1);
  // Serial.println(" ms");

  // Serial.print("Elapsed time for thermistor3: ");
  // Serial.print(end_time3 - end_time2);
  // Serial.println(" ms");

  // Serial.print("Total elapsed time: ");
  // Serial.print(test_end_time - test_start_time);
  // Serial.println(" ms");

  // Manually construct the JSON payload
  char payload[256];
  snprintf(payload, sizeof(payload),
    "{\"tempEngine\":%f, \"tempBattery\":%f, \"tempResistor\":%f}",
    temp_1, temp_2, temp_3);

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
  file.printf("%f,%f,%f,%f\n", 
              elapsedTime, temp_1, temp_2, temp_3);
  file.close();
}

String read_sensors() {
  // INA
  float busVoltage1 = INA1.readBusVoltage() / 1000.0;
  float busVoltage2 = INA2.readBusVoltage() / 1000.0;

  // Thermistors
  thermistor1.requestTemperatures();
  float temp_1 = thermistor1.getTempFByIndex(0);
  thermistor2.requestTemperatures();
  float temp_2 = thermistor2.getTempFByIndex(0);

  // Convert the sensor readings to JSON format
  String json = "{\"voltageAlt\":" + String(busVoltage1) + ",\"voltageBat\":" + String(busVoltage2) + ",\"tempEngine\":" + String(temp_1) + ",\"tempBattery\":" + String(temp_2)+ "}";
  return json;
}

// Control servo
void control_servo() {
  WiFiClient client = server.available(); 

  if (client) {
    currentTime = millis();
    previousTime = currentTime;
    String currentLine = "";
    while (client.connected() && currentTime - previousTime <= timeoutTime) {
        currentTime = millis();
        if (client.available()) {
            char c = client.read();
            header += c;
            if (c == '\n') {
                if (currentLine.length() == 0) {
                  if (header.indexOf("GET /sensorData") >= 0 ) {
                    String data = read_sensors(); //"{\"voltageAlt\":0.0,\"voltageBat\":0.0,\"tempEngine\":0.0,\"tempBattery\":0.0}";
                    client.println("HTTP/1.1 200 OK");
                    client.println("Content-type:application/json");
                    client.println("Connection: close");
                    client.println();
                    client.print(data);
                    break;
                  } else {
                      client.println("HTTP/1.1 200 OK");
                      client.println("Content-type:text/html");
                      client.println("Connection: close");
                      client.println();

                      // Display the HTML web page
                      client.println("<!DOCTYPE html>");
                      client.println("<html>");
                      client.println("<head>");
                      client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
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
                      client.println("<h1>Servo Control Center</h1>");
                      client.println("<p>Position: <span id=\"servoPos\"></span></p>");          
                      client.println("<input type=\"range\" min=\"40\" max=\"140\" class=\"slider\" id=\"servoSlider\" onchange=\"servo(this.value)\" value=\""+valueString+"\"/>"); // Edit
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
                      client.println("<br/><br/>");
                      client.println("<p>Alternator Voltage: <span id=\"alternatorVoltage\"></span> V</p>");
                      client.println("<p>Battery Voltage: <span id=\"batteryVoltage\"></span> V</p>");
                      client.println("<p>Engine Temperature: <span id=\"engineTemp\"></span> F</p>");
                      client.println("<p>Battery Temperature: <span id=\"batteryTemp\"></span> F</p>");
                      client.println("</div>");
                      client.println("<script src=\"https://ajax.googleapis.com/ajax/libs/jquery/3.3.1/jquery.min.js\"></script>");
                      client.println("<script>");
                      client.println("var slider = document.getElementById(\"servoSlider\");");
                      client.println("var servoP = document.getElementById(\"servoPos\"); servoP.innerHTML = slider.value;");
                      client.println("slider.oninput = function() { slider.value = this.value; servoP.innerHTML = this.value; servo(this.value); }");
                      client.println("$.ajaxSetup({timeout:1000}); function servo(pos) { ");
                      client.println("$.get(\"/?value=\" + pos + \"&\"); {Connection: close};}");
                      client.println("document.getElementById(\"upButton\").addEventListener(\"click\", function() { adjustServo(10); });");
                      client.println("document.getElementById(\"downButton\").addEventListener(\"click\", function() { adjustServo(-10); });");
                      client.println("document.getElementById(\"button_25\").addEventListener(\"click\", function() { slider.value = 65; servoP.innerHTML = 65; servo(65); });"); // Edit
                      client.println("document.getElementById(\"button_50\").addEventListener(\"click\", function() { slider.value = 90; servoP.innerHTML = 90; servo(90); });"); // Edit
                      client.println("document.getElementById(\"button_75\").addEventListener(\"click\", function() { slider.value = 115; servoP.innerHTML = 115; servo(115); });"); // Edit
                      client.println("document.getElementById(\"maxButton\").addEventListener(\"click\", function() { slider.value = 140; servoP.innerHTML = 140; servo(140); });"); // Edit
                      client.println("document.getElementById(\"killButton\").addEventListener(\"click\", function() { slider.value = 40; servoP.innerHTML = 40; servo(40); var audio = document.getElementById('killSound'); audio.play(); });"); // Edit
                      client.println("function adjustServo(offset) { var newValue = parseInt(slider.value) + offset;");
                      client.println("if (newValue < 40) { newValue = 40; }");   // Edit
                      client.println("if (newValue > 140) { newValue = 140; }"); // Edit
                      client.println("slider.value = newValue; servoP.innerHTML = newValue; servo(newValue); }");
                      
                      // Function to fetch and display sensor data
                      client.println("function fetchSensorData() {");
                      client.println("$.get('/sensorData', function(data) {");
                      client.println("$('#alternatorVoltage').text(data.voltageAlt);");
                      client.println("$('#batteryVoltage').text(data.voltageBat);");
                      client.println("$('#engineTemp').text(data.tempEngine);");
                      client.println("$('#batteryTemp').text(data.tempBattery);");
                      client.println("}, 'json');");
                      client.println("}");
                      client.println("setInterval(fetchSensorData, 1000);"); // Fetch data every second
                      
                      client.println("</script>");
                      client.println("</body>");
                      client.println("</html>");

                      //GET /?value=180& HTTP/1.1
                      if (header.indexOf("GET /?value=") >= 0) {
                          pos1 = header.indexOf('=');
                          pos2 = header.indexOf('&');
                          valueString = header.substring(pos1 + 1, pos2);

                          // Rotate the servo
                          servo.write(valueString.toInt());
                          //Serial.println(valueString);
                      }

                      client.println();
                      break;
                  }
                } else {
                    currentLine = "";
                }
            } else if (c != '\r') {
                currentLine += c;
            }
        }
    }
    header = "";
    client.stop();
  }
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
  unsigned long test_start_time = millis();
  control_servo(); 

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
        control_servo();
        reconnect();
        control_servo();
      }
      client.loop();
      tft.setCursor(10, 10);
      tft.setTextColor(TFT_WHITE);
      read_IMU();
      control_servo();
      read_BME();
      control_servo();
      read_INA();
      control_servo();
      //read_hall();
      //control_servo();
      read_therm();
    }
  }
  unsigned long test_end_time = millis();
  // Serial.print("Total elapsed time: ");
  // Serial.print(test_end_time - test_start_time);
  // Serial.println(" ms");
}

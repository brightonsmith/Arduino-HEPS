/*
Arduino/ESP 32 code for an HiL demo. This demo simulates a battery being recharged by a variable power supply and discharging into an arbitrary load. 
Voltage and temperature sensor data is read in by the ESP 32 and published to an MQTT broker (EMQX in this case).
The output of the system is a servo acting as a simulated engine throttle (for instance, if the variable power supply was replaced by an engine with an alternator, 
the servo would be used for adjusting the throttle of the engine and thus the voltage of the alternator.)

Hardware: 
- ESP 32 (Wroom, LilyGO T-Display, etc.) -> connects to all electronics
- One or Two INA 228 voltage sensors -> connected to ESP 32 and reading real voltage values from the power supply or the power supply AND battery
- One DS18B20 temperature probe -> connected to ESP 32 and reading temperature values from the battery (or whatever else)
- One MG90S servo (or comparable servo) -> connected to ESP 32 and adjusting position based on values read in from MQTT server

If using LilyGO T-Display, uncomment `// #include <TFT_eSPI.h>` as well as all `tft` lines to use the LCD display.

If using one INA 228 voltage sensor, additional voltage values will be simulated to act as a battery being discharged or charged, corresponding to the power supply.
If using two INA 228 voltage sensors, uncomment all `INA2` lines. One INA 228 will be connected to power supply, the other to a battery. 
*/

// #include <TFT_eSPI.h> // For LCD Display. Make sure to appropriately configure `Arduino/libraries/TFT_eSPI/User_Setup_Select.h` depending on the T-Display version
#include <Wire.h> // For I2C communication
#include <WiFi.h>
#include <PubSubClient.h> // For MQTT connection
#include <Adafruit_INA228.h> // Voltage sensor(s)
#include <OneWire.h> // Temperature probe
#include <DallasTemperature.h> // Temperature probe
#include <ArduinoJson.h> // Parsing JSON data (such as JSON data taken from an MQTT server)
#include <math.h> // Good to include
#include <SPIFFS.h> // For saving data to local ESP 32 memory
#include <ESP32Servo.h> // Must use this library for servo control

///////////////////////////////////////////////////////////////////////////// Config and Globals ////////////////////////////////////////////////////////////////////////////////////////////
// Pins
#define ONE_WIRE_1 32 // Battery therm
#define SERVO_PIN 25 // Servo
#define GPIO_0 0 // Interrupt (`BOOT` button on Wroom ESP 32)

// Function declarations
void IRAM_ATTR isr();

// WiFi credentials
const char *ssid = "your_SSID"; // Set by user
const char *password = "your_PASSWORD"; // Set by user

// MQTT Broker details
const char *mqtt_broker = "broker.emqx.io"; // Free public broker
String parent_topic = "parent/"; // Each piece of data published will use this prefix before the data-specific name (i.e. "parent/servo")
const int mqtt_port = 1883; // Unsecure port
String subscribe_topic = parent_topic + "servo";

WiFiClient espClient; 
PubSubClient client(espClient);

// TFT_eSPI tft = TFT_eSPI(); // TFT display instance

// Sensors
// INA
Adafruit_INA228 INA1; // Supply
Adafruit_INA228 INA2; // Battery

// Thermistor
OneWire oneWire1(ONE_WIRE_1);

DallasTemperature thermistor1(&oneWire1);

// Servo
Servo servo; // Servo instance

bool readCsvMode = false; // Flag to determine whether to read local data saved to ESP 32 or live sensor data
String cmdBuffer = ""; // For serial commands

unsigned long startTime; // Global variable to store the starting time

volatile bool isPaused = false; // Interrupt status (set to true to avoid immediate data flooding)

// Keep track of timing of recent interrupts
volatile unsigned long button_time = 0;
volatile unsigned long last_button_time = 0;

// Initialize variables for battery voltage simulation
float busVoltage2 = 0.0; // Simulated battery voltage
bool firstRun = true;    // Flag to initialize battery voltage on first run

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
  attachInterrupt(GPIO_0, isr, FALLING);
}

// MQTT callback function. This is how MQTT topics are subscribed to and the corresponding data is processed.
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic); // Subscribed to topic
  
  // Parse JSON (if expected data from the subscribed to topic is in JSON format)
  DynamicJsonDocument doc(256);
  deserializeJson(doc, payload, length);
  
  // Extract servoPos for this example
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

// Reconnect to MQTT when connection fails
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

// Headers for csv files
void write_headers() {
  fs::File INA_file = SPIFFS.open("/INA.csv", FILE_WRITE);
  if(!INA_file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  INA_file.println("time(s),supplyVoltage(V),batteryVoltage(V)");
  INA_file.close();

  fs::File therm_file = SPIFFS.open("/therm.csv", FILE_WRITE);
  if(!therm_file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  therm_file.println("time(s),batteryTemp(F)");
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
  if (!INA1.begin(0x40)) {
      Serial.println("Supply INA not found, check wiring, address, sensor ID!");
      while (1) delay(10);
  } 

  // // INA2 with address 0x41 (A0 = VCC, A1 = GND) 
  // if (!INA2.begin(0x41)) {
  //     Serial.println("Battery INA not found, check wiring, address, sensor ID!");
  //     while (1) delay(10);
  // }

  INA1.setShunt(0.1, 1.0);
  // INA2.setShunt(0.1, 1.0);

  INA1.setAveragingCount(INA228_COUNT_16);
  // INA2.setAveragingCount(INA228_COUNT_16);

  Serial.println("INA(s) found and initialized!");
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
  int pos;

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2000);

  // Sweep to ensure servo is working
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
		// in steps of 1 degree
		servo.write(pos);    // tell servo to go to position in variable 'pos'
		delay(15);             // waits 15ms for the servo to reach the position
	}
	for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
		servo.write(pos);    // tell servo to go to position in variable 'pos'
		delay(15);             // waits 15ms for the servo to reach the position
	}
  
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
    // tft.println("Press GPIO0 to start.");

    Serial.println("Clearing memory.");
    // tft.setCursor(10, 10);
    clear_memory("INA");
    clear_memory("therm");

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
  float busVoltage1 = INA1.readBusVoltage() / 1000.0; // Read supply voltage (V)
  // float busVoltage2 = INA2.readBusVoltage() / 1000.0; // Read battery voltage (V)
  
  // IF ONLY USING ONE VOLTAGE SENSOR
  // Initialize the battery voltage within 0.5 V of the supply's voltage on first run
  if (firstRun) {
    busVoltage2 = busVoltage1 + random(-5, 6) / 10.0; // Random initial difference between -0.5 and +0.5 V
    firstRun = false; // Set the flag to false after initialization
  }

  // Calculate voltage difference
  float voltageDifference = busVoltage1 - busVoltage2;

  // Determine how much to adjust the battery voltage based on the voltage difference
  // The adjustment factor determines how responsive the battery voltage is to changes in suppy voltage
  float adjustmentFactor = 0.007; // Adjust this factor for more or less responsiveness (higher means more resonsive)
  float adjustment = adjustmentFactor * voltageDifference;

  // Adjust the battery voltage, ensuring it remains within the specified range and within ±1 V of the supply voltage
  busVoltage2 += adjustment;
  busVoltage2 = constrain(busVoltage2, max(12.5, busVoltage1 - 1.0), min(14.4, busVoltage1 + 1.0));

  // Output the simulated voltages to the Serial Monitor
  Serial.print("Supply voltage: ");
  Serial.print(busVoltage1);
  Serial.println(" V");

  Serial.print("Battery voltage: ");
  Serial.print(busVoltage2);
  Serial.println(" V");

  // Manually construct the JSON payload for MQTT
  char payload[512];
  snprintf(payload, sizeof(payload),
    "{\"voltageAlt\":%.2f,\"voltageBat\":%.2f}",
    busVoltage1, busVoltage2);

  // Print the payload to Serial for debugging
  // Serial.println(payload);

  // Define the MQTT topic and publish the payload
  String INA_topic = parent_topic + "INA";
  client.publish(INA_topic.c_str(), payload);

  // Calculate the elapsed time in seconds for data logging
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - startTime) / 1000.0; // Convert milliseconds to seconds

  // Save data to CSV file on SPIFFS
  fs::File file = SPIFFS.open("/INA.csv", FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  file.printf("%.2f,%.2f,%.2f\n", 
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
            read_csv("IMU"); // MPU 9250
            cmdBuffer = "";
            break;
          case 'B':
            read_csv("BME"); // BME 280
            cmdBuffer = "";
            break;
          case 'V':
            read_csv("INA"); // INA 228
            cmdBuffer = "";
            break;
          case 'H':
            read_csv("hall"); // Hall effect sensor
            cmdBuffer = "";
            break;
          case 'T':
            read_csv("therm"); // DS18B20
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
    client.loop(); // Performs necessary MQTT callback protocols

    // tft.setCursor(10, 10);
    // tft.setTextColor(TFT_WHITE);

    read_INA();
    read_therm();
    Serial.println("----------------------------------------");

    // tft.fillScreen(TFT_BLACK);
    }
  }
}
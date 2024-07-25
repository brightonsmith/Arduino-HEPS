#include <SPIFFS.h>

void setup() {
  Serial.begin(115200);

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  // Attempt to delete IMU.csv if it exists
  if (SPIFFS.exists("/IMU.csv")) {
    Serial.println("IMU.csv exists. Attempting to delete...");
    if (SPIFFS.remove("/IMU.csv")) {
      Serial.println("IMU.csv successfully deleted.");
    } else {
      Serial.println("Failed to delete IMU.csv.");
    }
  } else {
    Serial.println("IMU.csv does not exist.");
  }

  // Verify whether IMU.csv has been deleted
  if (SPIFFS.exists("/IMU.csv")) {
    Serial.println("IMU.csv still exists. Deletion failed.");
  } else {
    Serial.println("IMU.csv does not exist. Deletion successful.");
  }
}

void loop() {
  // Nothing to do here
}

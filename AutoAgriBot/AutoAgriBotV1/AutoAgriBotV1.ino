/*
  ThingSpeak Integration for AutoAgriBot on ESP8266
  Sends soil moisture, temperature, and humidity to ThingSpeak.
  Adapted for DHT11 and soil moisture sensor.
*/

#include <ESP8266WiFi.h>
#include "ThingSpeak.h"
#include <DHT.h>

const char* ssid = "BOT-123";          // Replace with your Wi-Fi name
const char* password = "12345123";  // Replace with your Wi-Fi password

WiFiClient client;

unsigned long myChannelNumber = 3123787;  // Replace with your ThingSpeak channel number (e.g., 1234567)
const char* myWriteAPIKey = "5ZL8RKYCMPHTLSHQ";     // Replace with your Write API Key

// Timer variables (send data every 30 seconds)
unsigned long lastTime = 0;
unsigned long timerDelay = 1000;

// Sensor definitions
#define DHTPIN D7          // DHT11 data pin (GPIO4 / D2 on NodeMCU)
#define DHTTYPE DHT11     // DHT11 sensor type
DHT dht(DHTPIN, DHTTYPE);

#define SOIL_MOISTURE_PIN A0  // Soil moisture analog pin (A0)

// Variables for readings
float temperatureC;
float humidity;
float soilMoisturePercent;

void setup() {
  Serial.begin(115200);  // Initialize serial for debugging
  dht.begin();           // Initialize DHT11
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  ThingSpeak.begin(client);  // Initialize ThingSpeak
}
void loop() {
  if ((millis() - lastTime) > timerDelay) {
    
    // Read temperature and humidity from DHT11
    temperatureC = dht.readTemperature();
    if (isnan(temperatureC)) {
      Serial.println("Failed to read temperature!");
      temperatureC = 0;
    } else {
      Serial.print("Temperature (°C): ");
      Serial.println(temperatureC);
    }
    
    humidity = dht.readHumidity();
    if (isnan(humidity)) {
      Serial.println("Failed to read humidity!");
      humidity = 0;
    } else {
      Serial.print("Humidity (%): ");
      Serial.println(humidity);
    }
    
    // Read soil moisture and map to 0-24% based on your data
    int soilRaw = analogRead(SOIL_MOISTURE_PIN);
    float soilMoisturePercent = map(soilRaw, 1024, 830, 0, 10);  // 0% at 1024, 24% at 782
    if (soilMoisturePercent > 10) soilMoisturePercent = 10;      // Cap at 24%
    if (soilMoisturePercent < 0) soilMoisturePercent = 0;        // Cap at 0%
    // Normalize to 0-100% for ThingSpeak
    soilMoisturePercent = soilMoisturePercent * (100.0 / 10);  // Scale to 0-100%
    Serial.print("Soil Moisture (%): ");
    Serial.println(soilMoisturePercent);
    Serial.print("Soil Moisture (raw): ");
    Serial.println(soilRaw);

    // Set ThingSpeak fields
    ThingSpeak.setField(1, temperatureC);    // Field 1: Temperature
    ThingSpeak.setField(2, humidity);        // Field 2: Humidity
    ThingSpeak.setField(3, soilMoisturePercent);  // Field 3: Soil Moisture
    
    // Write to ThingSpeak
    int response = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (response == 200) {
      Serial.println("Data sent successfully.");
    } else {
      Serial.println("Error sending data. HTTP code: " + String(response));
    }
    
    lastTime = millis();
  }
  
  // Multi-level irrigation conditions based on soil moisture
  // if (soilMoisturePercent < 10 && temperatureC > 28) {  // Very dry (<10% ~ raw > 960)
  //   // Heavy watering: TURN ON water_pump for longer (e.g., 10 seconds)
  //   Serial.println("Heavy watering: Water pump ON (10s)");
  // } else if (soilMoisturePercent < 15 && temperatureC > 28) {  // Moderately dry (10-15% ~ raw 900-960)
  //   // Moderate watering: TURN ON water_pump for 5 seconds
  //   Serial.println("Moderate watering: Water pump ON (5s)");
  // } else if (soilMoisturePercent < 20 && temperatureC > 28) {  // Slightly dry (15-20% ~ raw 850-900)
  //   // Light watering: TURN ON water_pump for 2 seconds
  //   Serial.println("Light watering: Water pump ON (2s)");
  // } else {
  //   // Sufficient moisture: TURN OFF water_pump
  //   Serial.println("Water pump OFF");
  // }
}
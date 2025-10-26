#include <ESP8266WiFi.h>
#include <ThingSpeak.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

#define DHTPIN D7       // GPIO13 (D7) - DHT11
#define DHTTYPE DHT11
#define MOISTURE_PIN A0 // Analog pin for soil moisture
#define SERVO_PIN D0    // GPIO16 (D0) - Servo for moisture sensor
#define PUMP_PIN D5     // GPIO5 (D5) - Relay for water pump
#define MOISTURE_CHECK_INTERVAL 60000 // Check every 60s
#define IRRIGATION_DURATION 5000 // Irrigate for 5s
#define THINGSPEAK_INTERVAL 30000 // Send to ThingSpeak every 30s
#define MOISTURE_THRESHOLD 15.0 // Moisture threshold for irrigation (%)
#define TEMP_THRESHOLD 25.0     // Temperature threshold for irrigation (°C)

// WiFi and ThingSpeak settings
const char* ssid = "BOT-123"; // Replace with your Wi-Fi name
const char* password = "12345123"; // Replace with your Wi-Fi password
unsigned long myChannelNumber = 3123787; // Replace with your ThingSpeak channel number
const char* myWriteAPIKey = "5ZL8RKYCMPHTLSHQ"; // Replace with your Write API Key
WiFiServer server(80);
WiFiClient client;

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16x2 LCD
Servo moistureServo;
unsigned long lastSensorCheck = 0;
unsigned long lastThingSpeakUpdate = 0;

float lastTemperatureC = 0;
float lastTemperatureF = 0;
float lastHumidity = 0;

void setup() {
  Serial.begin(115200);
  pinMode(MOISTURE_PIN, INPUT);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, HIGH); // Relay OFF (active LOW)
  dht.begin();
  
  moistureServo.attach(SERVO_PIN, 500, 2400);
  moistureServo.write(90); // Center position
  delay(1000);

  Wire.begin(); // Default I2C pins: SDA (GPIO4/D2), SCL (GPIO5/D1)
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AutoAgriBot");
  delay(2000);
  lcd.clear();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("\nUnit 2 WiFi connected: " + WiFi.localIP().toString());
  
  ThingSpeak.begin(client); // Initialize ThingSpeak
  server.begin();
}

float readMoisture() {
  moistureServo.write(90); // Position sensor
  delay(500);
  int sensorValue = analogRead(MOISTURE_PIN);
  // Map to 0-24% based on provided calibration, then scale to 0-100%
  float moisturePercent = map(sensorValue, 1024, 830, 0, 24);
  if (moisturePercent > 24) moisturePercent = 24;
  if (moisturePercent < 0) moisturePercent = 0;
  moisturePercent = moisturePercent * (100.0 / 24); // Scale to 0-100%
  Serial.print("Moisture: "); Serial.print(moisturePercent); Serial.println("%");
  Serial.print("Moisture (raw): "); Serial.println(sensorValue);
  return moisturePercent;
}

void updateLCD(float tempC, float tempF, float hum) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(tempC, 1);
  lcd.print("C / ");
  lcd.print(tempF, 1);
  lcd.print("F");
  lcd.setCursor(0, 1);
  lcd.print("H:");
  lcd.print(hum, 1);
  lcd.print("% ");
  lcd.print(digitalRead(PUMP_PIN) == LOW ? "Pump:ON" : "Pump:OFF");
}

void uploadToThingSpeak(float temp, float hum, float moisture) {
  ThingSpeak.setField(1, temp); // Field 1: Temperature
  ThingSpeak.setField(2, hum); // Field 2: Humidity
  ThingSpeak.setField(3, moisture); // Field 3: Soil Moisture
  
  int response = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (response == 200) {
    Serial.println("Data sent successfully to ThingSpeak");
  } else {
    Serial.println("ThingSpeak upload failed: HTTP code " + String(response));
  }
}

void activatePump() {
  Serial.println("Starting irrigation");
  digitalWrite(PUMP_PIN, LOW); // Relay ON
  updateLCD(lastTemperatureC, lastTemperatureF, lastHumidity); // Update LCD to show Pump:ON
  delay(IRRIGATION_DURATION);
  digitalWrite(PUMP_PIN, HIGH); // Relay OFF
  Serial.println("Irrigation stopped");
  updateLCD(lastTemperatureC, lastTemperatureF, lastHumidity); // Update LCD to show Pump:OFF
}

void checkIrrigationCondition(float moisture, float temperatureC) {
  if (moisture < MOISTURE_THRESHOLD && temperatureC > TEMP_THRESHOLD) {
    Serial.println("Moisture < 15% and Temp > 25C - Activating pump");
    activatePump();
  } else {
    Serial.println("Irrigation not needed (Moisture: " + String(moisture) + "%, Temp: " + String(temperatureC) + "C)");
  }
}

void loop() {
  // Handle client requests
  WiFiClient client = server.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
        String request = client.readStringUntil('\r');
        if (request.indexOf("/moisture") != -1) {
          float moisture = readMoisture();
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/plain");
          client.println("Connection: close");
          client.println();
          client.println("MOISTURE:" + String(moisture));
        } else if (request.indexOf("/irrigate") != -1) {
          activatePump();
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/plain");
          client.println("Connection: close");
          client.println();
          client.println("Irrigation activated");
        }
        break;
      }
    }
    client.stop();
  }

  // Periodic sensor reading
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorCheck >= MOISTURE_CHECK_INTERVAL) {
    float humidity = dht.readHumidity();
    float temperatureC = dht.readTemperature();
    float temperatureF = dht.readTemperature(true);
    float moisture = readMoisture();
    
    if (!isnan(humidity) && !isnan(temperatureC) && !isnan(temperatureF)) {
      Serial.print("Temperature: "); Serial.print(temperatureC); Serial.print(" °C  ~  "); Serial.print(temperatureF); Serial.println(" °F");
      Serial.print("Humidity: "); Serial.print(humidity); Serial.println("%");
      lastTemperatureC = temperatureC; // Store values for use in activatePump
      lastTemperatureF = temperatureF;
      lastHumidity = humidity;
      updateLCD(temperatureC, temperatureF, humidity);
      checkIrrigationCondition(moisture, temperatureC); // Check irrigation condition
    } else {
      Serial.println("❌ Sensor Error!");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sensor Error!");
      temperatureC = 0;
      humidity = 0;
      temperatureF = 0;
      lastTemperatureC = 0; // Reset stored values
      lastTemperatureF = 0;
      lastHumidity = 0;
    }
    
    // Update ThingSpeak
    if (currentMillis - lastThingSpeakUpdate >= THINGSPEAK_INTERVAL) {
      uploadToThingSpeak(temperatureC, humidity, moisture);
      lastThingSpeakUpdate = currentMillis;
    }
    
    lastSensorCheck = currentMillis;
  }
}
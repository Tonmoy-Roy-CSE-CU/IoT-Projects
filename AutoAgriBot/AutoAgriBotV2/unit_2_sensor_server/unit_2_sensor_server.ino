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
#define SERVO_DELAY 1500       // Servo settling delay (ms)

// WiFi and ThingSpeak settings
const char* ssid = "BOT-123";
const char* password = "12345123";
unsigned long myChannelNumber = 3123787;
const char* myWriteAPIKey = "5ZL8RKYCMPHTLSHQ";
WiFiServer server(80);
WiFiClient client;

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo moistureServo;
unsigned long lastSensorCheck = 0;
unsigned long lastThingSpeakUpdate = 0;
unsigned long pumpStartTime = 0;
bool pumpRunning = false;
bool servoAtReadingPosition = false; // Track servo position
float lastTemperatureC = 0;
float lastTemperatureF = 0;
float lastHumidity = 0;
float lastMoisture = -1;

void setup() {
  Serial.begin(115200);
  pinMode(MOISTURE_PIN, INPUT);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, HIGH); // Relay OFF (active LOW)
  dht.begin();
  
  moistureServo.attach(SERVO_PIN, 500, 2400);
  Serial.println("Unit 2: Servo initialized");
  moistureServo.write(15); // Initial position
  servoAtReadingPosition = false;
  Serial.println("Unit 2: Servo set to 15 degrees (initial position)");
  delay(SERVO_DELAY);
  
  // Test servo movement
  Serial.println("Unit 2: Testing servo...");
  moistureServo.write(110);
  Serial.println("Unit 2: Servo test - moved to 110 degrees");
  delay(SERVO_DELAY);
  moistureServo.write(15);
  Serial.println("Unit 2: Servo test - returned to 15 degrees");
  delay(SERVO_DELAY);

  Wire.begin(); // Default I2C pins: SDA (GPIO4/D2), SCL (GPIO5/D1)
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AutoAgriBot v2");
  delay(2000);
  lcd.clear();

  // Set static IP
  IPAddress local_IP(10, 138, 17, 246);
  IPAddress gateway(10, 138, 17, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(local_IP, gateway, subnet);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nUnit 2 WiFi connected: " + WiFi.localIP().toString());
  
  ThingSpeak.begin(client);
  server.begin();
  Serial.println("Unit 2: Server started on port 80");
  Serial.println("Ready to receive requests from Unit 1");
}

float readMoisture() {
  // Move servo to reading position if not already there
  if (!servoAtReadingPosition) {
    Serial.println("→ Moving servo to 110° for moisture reading");
    moistureServo.write(110);
    delay(SERVO_DELAY);
    servoAtReadingPosition = true;
  } else {
    Serial.println("→ Servo already at 110° (reading position)");
  }
  
  Serial.println("→ Reading moisture sensor...");
  int sensorValue = analogRead(MOISTURE_PIN);
  float moisturePercent = map(sensorValue, 1024, 830, 0, 24);
  if (moisturePercent > 24) moisturePercent = 24;
  if (moisturePercent < 0) moisturePercent = 0;
  moisturePercent = moisturePercent * (100.0 / 24); // Scale to 0-100%
  lastMoisture = moisturePercent;
  Serial.print("→ Moisture: "); Serial.print(moisturePercent); Serial.print("%");
  Serial.print(" (raw ADC: "); Serial.print(sensorValue); Serial.println(")");
  return moisturePercent;
}

void returnServoToInitial() {
  if (servoAtReadingPosition) {
    Serial.println("→ Returning servo to 15° (initial position)");
    moistureServo.write(15);
    delay(SERVO_DELAY);
    servoAtReadingPosition = false;
    Serial.println("→ Servo returned successfully");
  } else {
    Serial.println("→ Servo already at initial position");
  }
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
  lcd.print(digitalRead(PUMP_PIN) == LOW ? "P:ON" : "P:OFF");
}

void uploadToThingSpeak(float temp, float hum, float moisture) {
  ThingSpeak.setField(1, temp);
  ThingSpeak.setField(2, hum);
  ThingSpeak.setField(3, moisture);
  
  int response = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  if (response == 200) {
    Serial.println("✓ ThingSpeak upload successful");
  } else {
    Serial.println("✗ ThingSpeak upload failed (code: " + String(response) + ")");
  }
}

void activatePump() {
  Serial.println("═══ STARTING IRRIGATION ═══");
  digitalWrite(PUMP_PIN, LOW); // Relay ON (active LOW)
  updateLCD(lastTemperatureC, lastTemperatureF, lastHumidity);
  pumpStartTime = millis();
  pumpRunning = true;
}

void stopPump() {
  digitalWrite(PUMP_PIN, HIGH); // Relay OFF
  pumpRunning = false;
  Serial.println("═══ IRRIGATION STOPPED ═══");
  updateLCD(lastTemperatureC, lastTemperatureF, lastHumidity);
}

bool checkIrrigationCondition(float moisture, float temperatureC) {
  Serial.println("Checking irrigation conditions:");
  Serial.println("  • Moisture: " + String(moisture) + "% (need: < " + String(MOISTURE_THRESHOLD) + "%)");
  Serial.println("  • Temperature: " + String(temperatureC) + "°C (need: > " + String(TEMP_THRESHOLD) + "°C)");
  
  if (moisture < MOISTURE_THRESHOLD && temperatureC > TEMP_THRESHOLD) {
    Serial.println("  ✓ Both conditions met - activating pump");
    activatePump();
    return true;
  } else {
    Serial.println("  ✗ Conditions NOT met - no irrigation");
    if (moisture >= MOISTURE_THRESHOLD) {
      Serial.println("    Reason: Moisture sufficient");
    }
    if (temperatureC <= TEMP_THRESHOLD) {
      Serial.println("    Reason: Temperature too low");
    }
    return false;
  }
}

void loop() {
  // Handle client requests from Unit 1
  WiFiClient client = server.available();
  if (client) {
    Serial.println("\n╔═══════════════════════════════╗");
    Serial.println("║  Client Connected (Unit 1)   ║");
    Serial.println("╚═══════════════════════════════╝");
    
    unsigned long timeout = millis() + 5000;
    String request = "";
    
    // Read request
    while (client.connected() && millis() < timeout) {
      if (client.available()) {
        String line = client.readStringUntil('\r');
        if (line.indexOf("GET") != -1) {
          request = line;
          break;
        }
      }
      delay(10);
    }
    
    Serial.println("Request: " + request);
    
    // ============================================
    // ENDPOINT 1: /sensordata
    // Returns moisture and temperature
    // Moves servo to 110° and KEEPS it there
    // ============================================
    if (request.indexOf("/sensordata") != -1) {
      Serial.println("\n[Endpoint: /sensordata]");
      
      // Read moisture (moves servo to 110° if needed)
      float moisture = readMoisture();
      
      // Read temperature
      float temperatureC = dht.readTemperature();
      if (isnan(temperatureC)) {
        Serial.println("⚠ DHT sensor error, using cached value");
        temperatureC = lastTemperatureC;
      } else {
        Serial.println("→ Temperature: " + String(temperatureC) + "°C");
      }
      
      // Send response
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println("Connection: close");
      client.println();
      client.println("MOISTURE:" + String(moisture));
      client.println("TEMP:" + String(temperatureC));
      
      Serial.println("✓ Sent sensor data to Unit 1");
      Serial.println("  MOISTURE: " + String(moisture) + "%");
      Serial.println("  TEMP: " + String(temperatureC) + "°C");
      Serial.println("⚠ Servo remains at 110° (waiting for /irrigate)");
    }
    
    // ============================================
    // ENDPOINT 2: /irrigate
    // Checks conditions and activates pump if needed
    // ALWAYS returns servo to 15° at the end
    // ============================================
    else if (request.indexOf("/irrigate") != -1) {
      Serial.println("\n[Endpoint: /irrigate]");
      
      // Get current temperature
      float temperatureC = dht.readTemperature();
      if (isnan(temperatureC)) {
        temperatureC = lastTemperatureC;
      }
      
      // Check if we have valid moisture data
      if (lastMoisture < 0) {
        Serial.println("⚠ No moisture data available, reading now...");
        lastMoisture = readMoisture();
      }
      
      // Check irrigation conditions
      bool irrigationActivated = checkIrrigationCondition(lastMoisture, temperatureC);
      
      if (irrigationActivated) {
        // Wait for pump to complete
        Serial.println("⏳ Waiting for pump cycle (" + String(IRRIGATION_DURATION) + "ms)...");
        unsigned long pumpWaitStart = millis();
        while (pumpRunning && (millis() - pumpStartTime) < IRRIGATION_DURATION) {
          delay(100);
          // Print progress every second
          if ((millis() - pumpWaitStart) % 1000 == 0) {
            unsigned long elapsed = millis() - pumpStartTime;
            Serial.println("  Pump running: " + String(elapsed) + "ms / " + String(IRRIGATION_DURATION) + "ms");
          }
        }
        stopPump();
      }
      
      // ALWAYS return servo to initial position
      returnServoToInitial();
      
      // Send response
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println("Connection: close");
      client.println();
      client.println("Irrigation completed");
      
      Serial.println("✓ Sent completion response to Unit 1");
      if (irrigationActivated) {
        Serial.println("  Irrigation cycle: COMPLETED");
      } else {
        Serial.println("  Irrigation cycle: SKIPPED (conditions not met)");
      }
    }
    
    // Unknown endpoint
    else {
      Serial.println("✗ Unknown endpoint");
      client.println("HTTP/1.1 404 Not Found");
      client.println();
    }
    
    client.stop();
    Serial.println("╚═══════════════════════════════╝");
    Serial.println("Client Disconnected\n");
  }

  // ============================================
  // Handle pump timeout (safety mechanism)
  // ============================================
  if (pumpRunning && (millis() - pumpStartTime) >= IRRIGATION_DURATION) {
    Serial.println("⚠ Pump timeout reached (safety stop)");
    stopPump();
  }

  // ============================================
  // Periodic sensor reading (independent)
  // Only runs when servo is NOT being used by Unit 1
  // ============================================
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorCheck >= MOISTURE_CHECK_INTERVAL) {
    Serial.println("\n╔════════════════════════════════╗");
    Serial.println("║  Periodic Sensor Check        ║");
    Serial.println("╚════════════════════════════════╝");
    
    // Read DHT sensor
    float humidity = dht.readHumidity();
    float temperatureC = dht.readTemperature();
    float temperatureF = dht.readTemperature(true);
    
    // Read moisture only if servo is free (not being used by Unit 1)
    float moisture = lastMoisture;
    if (!servoAtReadingPosition && !pumpRunning) {
      Serial.println("Servo is free - reading moisture");
      moisture = readMoisture();
      returnServoToInitial(); // Return immediately after periodic check
    } else {
      Serial.println("Servo in use - skipping moisture reading");
      Serial.println("Using cached value: " + String(lastMoisture) + "%");
    }
    
    // Validate and update DHT readings
    if (!isnan(humidity) && !isnan(temperatureC) && !isnan(temperatureF)) {
      Serial.println("DHT Readings:");
      Serial.println("  • Temperature: " + String(temperatureC) + "°C / " + String(temperatureF) + "°F");
      Serial.println("  • Humidity: " + String(humidity) + "%");
      
      lastTemperatureC = temperatureC;
      lastTemperatureF = temperatureF;
      lastHumidity = humidity;
      updateLCD(temperatureC, temperatureF, humidity);
      
      // Autonomous irrigation check (only if not already irrigating)
      if (!servoAtReadingPosition && !pumpRunning && moisture >= 0) {
        Serial.println("\nAutonomous irrigation check:");
        checkIrrigationCondition(moisture, temperatureC);
      }
    } else {
      Serial.println("✗ DHT Sensor Error!");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sensor Error!");
    }
    
    // Upload to ThingSpeak
    if (currentMillis - lastThingSpeakUpdate >= THINGSPEAK_INTERVAL) {
      Serial.println("\nUploading to ThingSpeak...");
      uploadToThingSpeak(lastTemperatureC, lastHumidity, moisture);
      lastThingSpeakUpdate = currentMillis;
    }
    
    lastSensorCheck = currentMillis;
    Serial.println("╚════════════════════════════════╝\n");
  }
}
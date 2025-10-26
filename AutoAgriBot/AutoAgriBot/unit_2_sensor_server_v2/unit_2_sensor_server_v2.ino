#include <ESP8266WiFi.h>
#include <ThingSpeak.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

#define DHTPIN D7
#define DHTTYPE DHT11
#define MOISTURE_PIN A0
#define SERVO_PIN D0
#define PUMP_PIN D5
#define MOISTURE_CHECK_INTERVAL 60000
#define IRRIGATION_DURATION 5000
#define THINGSPEAK_INTERVAL 20000
#define MOISTURE_THRESHOLD 15.0
#define TEMP_THRESHOLD 25.0
#define SERVO_DELAY 1500

// WiFi and ThingSpeak
const char* ssid = "BOT-123";
const char* password = "12345123";
unsigned long myChannelNumber = 3123787;
const char* myWriteAPIKey = "5ZL8RKYCMPHTLSHQ";

WiFiServer server(80);
WiFiClient tsClient;  // Dedicated ThingSpeak client

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo moistureServo;

unsigned long lastSensorCheck = 0;
unsigned long lastThingSpeakUpdate = 0;
unsigned long pumpStartTime = 0;
bool pumpRunning = false;
bool servoAtReadingPosition = false;
bool serverBusy = false;
float lastTemperatureC = 0;
float lastHumidity = 0;
float lastMoisture = -1;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== AutoAgriBot Unit 2 (Fixed) ===");
  
  pinMode(MOISTURE_PIN, INPUT);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, HIGH);
  dht.begin();
  
  moistureServo.attach(SERVO_PIN, 500, 2400);
  moistureServo.write(15);
  servoAtReadingPosition = false;
  Serial.println("Servo at 15°");
  delay(SERVO_DELAY);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("AutoAgriBot v2");
  delay(2000);
  lcd.clear();

  IPAddress local_IP(10, 138, 17, 246);
  IPAddress gateway(10, 138, 17, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(local_IP, gateway, subnet);
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nWiFi: " + WiFi.localIP().toString());
  
  ThingSpeak.begin(tsClient);
  Serial.println("ThingSpeak initialized");
  
  server.begin();
  Serial.println("Server started\n");
}

float readMoisture() {
  if (!servoAtReadingPosition) {
    Serial.println("→ Servo to 110°");
    moistureServo.write(110);
    delay(SERVO_DELAY);
    servoAtReadingPosition = true;
  }
  
  int raw = analogRead(MOISTURE_PIN);
  float moisture = map(raw, 1024, 830, 0, 24);
  if (moisture > 24) moisture = 24;
  if (moisture < 0) moisture = 0;
  moisture = moisture * (100.0 / 24);
  lastMoisture = moisture;
  Serial.println("→ Moisture: " + String(moisture) + "% (raw: " + String(raw) + ")");
  return moisture;
}

void returnServoToInitial() {
  if (servoAtReadingPosition) {
    Serial.println("→ Servo to 15°");
    moistureServo.write(15);
    delay(SERVO_DELAY);
    servoAtReadingPosition = false;
  }
}

void updateLCD(float tempC, float hum, float moisture) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(tempC, 1);
  lcd.print("C M:");
  lcd.print(moisture, 0);  // No decimal for moisture to save space
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("H:");
  lcd.print(hum, 0);  // No decimal for humidity to save space
  lcd.print("% ");
  lcd.print(digitalRead(PUMP_PIN) == LOW ? "Pump:ON" : "Pump:OFF");
}

void uploadToThingSpeak(float temp, float hum, float moisture) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("✗ WiFi down");
    return;
  }
  
  if (serverBusy || pumpRunning || servoAtReadingPosition) {
    Serial.println("⚠️ System busy - skip upload");
    return;
  }
  
  unsigned long timeSince = millis() - lastThingSpeakUpdate;
  if (timeSince < 15000) {
    return; // Skip silently if too soon
  }
  
  Serial.println("\n╔════════════════════════════════╗");
  Serial.println("║  ThingSpeak Upload            ║");
  Serial.println("╚════════════════════════════════╝");
  
  // Stop server temporarily
  server.stop();
  delay(500);
  
  // Clean up client
  tsClient.stop();
  delay(1000);
  
  Serial.println("Uploading...");
  
  ThingSpeak.setField(1, temp);
  ThingSpeak.setField(2, hum);
  ThingSpeak.setField(3, moisture);
  
  int response = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  
  if (response == 200) {
    Serial.println("✓ SUCCESS!");
    lastThingSpeakUpdate = millis();
  } else {
    Serial.println("✗ Error: " + String(response));
    if (response == -301) Serial.println("  Connection failed");
    else if (response == -302) Serial.println("  Rate limit");
    else if (response == -304) Serial.println("  Timeout");
    lastThingSpeakUpdate = millis(); // Update anyway to prevent spam
  }
  
  // Restart server
  delay(500);
  server.begin();
  Serial.println("Server restarted");
  Serial.println("╚════════════════════════════════╝\n");
}

void activatePump() {
  Serial.println("═══ PUMP ON ═══");
  digitalWrite(PUMP_PIN, LOW);
  updateLCD(lastTemperatureC, lastHumidity, lastMoisture);
  pumpStartTime = millis();
  pumpRunning = true;
}

void stopPump() {
  digitalWrite(PUMP_PIN, HIGH);
  pumpRunning = false;
  Serial.println("═══ PUMP OFF ═══");
  updateLCD(lastTemperatureC, lastHumidity, lastMoisture);
}

bool checkIrrigationCondition(float moisture, float temperatureC) {
  Serial.println("Checking irrigation:");
  Serial.println("  M: " + String(moisture) + "% (need < " + String(MOISTURE_THRESHOLD) + "%)");
  Serial.println("  T: " + String(temperatureC) + "°C (need > " + String(TEMP_THRESHOLD) + "°C)");
  
  if (moisture < MOISTURE_THRESHOLD && temperatureC > TEMP_THRESHOLD) {
    Serial.println("  ✓ Conditions met - activating");
    activatePump();
    return true;
  } else {
    Serial.println("  ✗ Not needed");
    return false;
  }
}

void loop() {
  WiFiClient serverClient = server.available();
  
  if (serverClient) {
    serverBusy = true;
    Serial.println("\n╔═══════════════════════════════╗");
    Serial.println("║  Client Connected            ║");
    Serial.println("╚═══════════════════════════════╝");
    
    unsigned long timeout = millis() + 5000;
    String request = "";
    
    while (serverClient.connected() && millis() < timeout) {
      if (serverClient.available()) {
        String line = serverClient.readStringUntil('\r');
        if (line.indexOf("GET") != -1) {
          request = line;
          break;
        }
      }
      delay(10);
    }
    
    Serial.println("Request: " + request);
    
    if (request.indexOf("/sensordata") != -1) {
      Serial.println("[/sensordata]");
      float moisture = readMoisture();
      float temperatureC = dht.readTemperature();
      if (isnan(temperatureC)) {
        temperatureC = lastTemperatureC;
        Serial.println("⚠️ Using cached temp");
      }
      
      serverClient.println("HTTP/1.1 200 OK");
      serverClient.println("Content-Type: text/plain");
      serverClient.println("Connection: close");
      serverClient.println();
      serverClient.println("MOISTURE:" + String(moisture));
      serverClient.println("TEMP:" + String(temperatureC));
      Serial.println("✓ Sent");
    }
    else if (request.indexOf("/irrigate") != -1) {
      Serial.println("[/irrigate]");
      float temperatureC = dht.readTemperature();
      if (isnan(temperatureC)) temperatureC = lastTemperatureC;
      if (lastMoisture < 0) lastMoisture = readMoisture();
      
      bool irrigated = checkIrrigationCondition(lastMoisture, temperatureC);
      if (irrigated) {
        while (pumpRunning && (millis() - pumpStartTime) < IRRIGATION_DURATION) {
          delay(100);
        }
        stopPump();
      }
      
      returnServoToInitial();
      
      serverClient.println("HTTP/1.1 200 OK");
      serverClient.println("Content-Type: text/plain");
      serverClient.println("Connection: close");
      serverClient.println();
      serverClient.println("Irrigation completed");
      Serial.println("✓ Done");
    }
    
    serverClient.stop();
    serverBusy = false;
    Serial.println("╚═══════════════════════════════╝\n");
    
    delay(2000); // Wait before allowing ThingSpeak
  }

  // Pump safety
  if (pumpRunning && (millis() - pumpStartTime) >= IRRIGATION_DURATION) {
    stopPump();
  }

  // Periodic sensor check
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorCheck >= MOISTURE_CHECK_INTERVAL) {
    if (!serverBusy && !pumpRunning) {
      Serial.println("\n╔════════════════════════════════╗");
      Serial.println("║  Periodic Check               ║");
      Serial.println("╚════════════════════════════════╝");
      
      float humidity = dht.readHumidity();
      float temperatureC = dht.readTemperature();
      
      float moisture = lastMoisture;
      if (!servoAtReadingPosition && !pumpRunning) {
        moisture = readMoisture();
        returnServoToInitial();
      } else {
        Serial.println("Servo busy - using cached");
      }
      
      if (!isnan(humidity) && !isnan(temperatureC)) {
        Serial.println("T=" + String(temperatureC) + "°C H=" + String(humidity) + "% M=" + String(moisture) + "%");
        lastTemperatureC = temperatureC;
        lastHumidity = humidity;
        updateLCD(temperatureC, humidity, moisture);
        
        if (!servoAtReadingPosition && !pumpRunning && moisture >= 0) {
          checkIrrigationCondition(moisture, temperatureC);
        }
      } else {
        Serial.println("✗ DHT Error");
        lcd.clear();
        lcd.print("Sensor Error!");
      }
      
      lastSensorCheck = currentMillis;
      Serial.println("╚════════════════════════════════╝\n");
    } else {
      Serial.println("⚠️ Skip periodic - system busy");
      lastSensorCheck = currentMillis;
    }
  }
  
  // SEPARATE ThingSpeak upload check (runs independently)
  if (!serverBusy && !pumpRunning && !servoAtReadingPosition) {
    if (currentMillis - lastThingSpeakUpdate >= THINGSPEAK_INTERVAL) {
      if (lastTemperatureC > 0 && lastHumidity > 0 && lastMoisture >= 0) {
        uploadToThingSpeak(lastTemperatureC, lastHumidity, lastMoisture);
      }
    }
  }
}
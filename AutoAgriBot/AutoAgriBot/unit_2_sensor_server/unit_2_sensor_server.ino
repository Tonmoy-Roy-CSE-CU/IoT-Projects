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

// CRITICAL FIX: Use DIFFERENT variable names to avoid shadowing!
WiFiClient tsClient;  // For ThingSpeak ONLY - renamed to avoid conflict!

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo moistureServo;

unsigned long lastSensorCheck = 0;
unsigned long lastThingSpeakUpdate = 0;
unsigned long pumpStartTime = 0;
bool pumpRunning = false;
bool servoAtReadingPosition = false;
float lastTemperatureC = 0;
float lastTemperatureF = 0;
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

  // Set static IP
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
  
  // CRITICAL: Initialize ThingSpeak with dedicated client
  ThingSpeak.begin(tsClient);
  Serial.println("ThingSpeak initialized with dedicated client");
  
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
  Serial.println("→ Moisture: " + String(moisture) + "%");
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

void updateLCD(float tempC, float tempF, float hum) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(tempC, 1);
  lcd.print("C ");
  lcd.print(tempF, 1);
  lcd.print("F");
  lcd.setCursor(0, 1);
  lcd.print("H:");
  lcd.print(hum, 1);
  lcd.print("% ");
  lcd.print(digitalRead(PUMP_PIN) == LOW ? "P:ON" : "P:OFF");
}

void uploadToThingSpeak(float temp, float hum, float moisture) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("✗ WiFi down");
    return;
  }
  
  unsigned long timeSince = millis() - lastThingSpeakUpdate;
  if (timeSince < 15000) {
    Serial.println("⚠️ Waiting " + String(15000 - timeSince) + "ms");
    delay(15000 - timeSince);
  }
  
  Serial.println("Uploading to ThingSpeak...");
  
  ThingSpeak.setField(1, temp);
  ThingSpeak.setField(2, hum);
  ThingSpeak.setField(3, moisture);
  
  int response = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  
  if (response == 200) {
    Serial.println("✓ SUCCESS!");
  } else {
    Serial.println("✗ Error: " + String(response));
    if (response == -301) Serial.println("  Connection failed");
    else if (response == -302) Serial.println("  Rate limit");
    else if (response == -304) Serial.println("  Timeout");
  }
  
  lastThingSpeakUpdate = millis();
}

void activatePump() {
  Serial.println("═══ PUMP ON ═══");
  digitalWrite(PUMP_PIN, LOW);
  updateLCD(lastTemperatureC, lastTemperatureF, lastHumidity);
  pumpStartTime = millis();
  pumpRunning = true;
}

void stopPump() {
  digitalWrite(PUMP_PIN, HIGH);
  pumpRunning = false;
  Serial.println("═══ PUMP OFF ═══");
  updateLCD(lastTemperatureC, lastTemperatureF, lastHumidity);
}

bool checkIrrigationCondition(float moisture, float temperatureC) {
  if (moisture < MOISTURE_THRESHOLD && temperatureC > TEMP_THRESHOLD) {
    Serial.println("✓ Irrigating (M=" + String(moisture) + "% T=" + String(temperatureC) + "°C)");
    activatePump();
    return true;
  }
  return false;
}

void loop() {
  // CRITICAL FIX: Use DIFFERENT variable name for server client!
  WiFiClient serverClient = server.available();  // ← NOT "client"!
  
  if (serverClient) {
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
      if (isnan(temperatureC)) temperatureC = lastTemperatureC;
      
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
    
    serverClient.stop();  // Close server client only
    Serial.println("╚═══════════════════════════════╝\n");
  }

  // Pump safety
  if (pumpRunning && (millis() - pumpStartTime) >= IRRIGATION_DURATION) {
    stopPump();
  }

  // Periodic check
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorCheck >= MOISTURE_CHECK_INTERVAL) {
    Serial.println("\n╔════════════════════════════════╗");
    Serial.println("║  Periodic Check               ║");
    Serial.println("╚════════════════════════════════╝");
    
    float humidity = dht.readHumidity();
    float temperatureC = dht.readTemperature();
    float temperatureF = dht.readTemperature(true);
    
    float moisture = lastMoisture;
    if (!servoAtReadingPosition && !pumpRunning) {
      moisture = readMoisture();
      returnServoToInitial();
    } else {
      Serial.println("Servo busy - cached");
    }
    
    if (!isnan(humidity) && !isnan(temperatureC)) {
      Serial.println("T=" + String(temperatureC) + "°C H=" + String(humidity) + "%");
      lastTemperatureC = temperatureC;
      lastTemperatureF = temperatureF;
      lastHumidity = humidity;
      updateLCD(temperatureC, temperatureF, humidity);
      
      if (!servoAtReadingPosition && !pumpRunning && moisture >= 0) {
        checkIrrigationCondition(moisture, temperatureC);
      }
    } else {
      Serial.println("✗ DHT Error");
    }
    
    // ThingSpeak upload
    if (currentMillis - lastThingSpeakUpdate >= THINGSPEAK_INTERVAL) {
      uploadToThingSpeak(lastTemperatureC, lastHumidity, moisture);
    }
    
    lastSensorCheck = currentMillis;
    Serial.println("╚════════════════════════════════╝\n");
  }
}
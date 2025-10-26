#include <ESP8266WiFi.h>
#include <Servo.h>

#define TRIG_PIN D7    // Ultrasonic Sensor TRIG
#define ECHO_PIN D8    // Ultrasonic Sensor ECHO
#define SERVO_PIN D0   // Servo for Ultrasonic
#define ENA D6         // Motor A Enable
#define IN1 D1         // Motor A Input 1
#define IN2 D2         // Motor A Input 2
#define IN3 D3         // Motor B Input 1
#define IN4 D4         // Motor B Input 2
#define ENB D5         // Motor B Enable

// WiFi and Server settings
const char* ssid = "BOT-123";
const char* password = "12345123";
const char* serverIP = "10.138.17.246"; // Unit 2 IP
const int serverPort = 80;

// Constants
#define MAX_DISTANCE 200       // Max sensor distance (cm)
#define OBSTACLE_DISTANCE 35   // Stop if obstacle within 35cm
#define TURN_DISTANCE 45       // Scan if obstacle within 45cm
#define EMERGENCY_DISTANCE 20  // Emergency reverse if within 20cm
#define SIDE_CLEAR_DISTANCE 50 // Min clear distance for sides
#define NORMAL_SPEED 90        // Normal speed (0-255)
#define TURN_SPEED 140         // Turn speed
#define BACKWARD_SPEED 90      // Backward speed
#define EMERGENCY_SPEED 90     // Emergency speed
#define SCAN_DELAY 250         // Servo delay (ms)
#define TURN_TIME_BASE 800     // Base turn time (ms)
#define LOOP_INTERVAL 50       // Loop interval (ms)
#define PULSE_TIMEOUT 30000    // pulseIn timeout (us)
#define MOISTURE_CHECK_INTERVAL 30000 // Check moisture every 30s
#define MOISTURE_THRESHOLD 15.0 // Moisture % below which irrigation is requested
#define TEMP_THRESHOLD 25.0     // Temperature threshold for irrigation (°C)
#define HTTP_TIMEOUT 10000     // HTTP request timeout (ms)
#define DATA_COLLECTION_WAIT 3000 // Wait for data collection only (ms)
#define IRRIGATION_WAIT 10000   // Wait for irrigation process (ms)

Servo sensorServo;
float distance;
int stuckCounter = 0;
int lastTurnDirection = 0; // 1 = right, -1 = left
unsigned long lastLoopTime = 0;
unsigned long lastMoistureCheck = 0;
bool emergencyMode = false;

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== AutoAgriBot Unit 1 (Car) ===");
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // CRITICAL: Stop motors before servo initialization
  carStop();
  delay(500);

  // Initialize ultrasonic servo
  sensorServo.attach(SERVO_PIN, 500, 2400);
  sensorServo.write(90); // Center servo
  Serial.println("Ultrasonic servo initialized at 90°");
  delay(1000);
  
  // Test servo movement
  Serial.println("Testing ultrasonic servo...");
  sensorServo.write(20);  // Right
  delay(500);
  Serial.println("  Servo at 20° (right)");
  sensorServo.write(160); // Left
  delay(500);
  Serial.println("  Servo at 160° (left)");
  sensorServo.write(90);  // Center
  delay(500);
  Serial.println("  Servo returned to 90° (center)");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nUnit 1 WiFi connected: " + WiFi.localIP().toString());
  Serial.println("Ready to operate!");
  Serial.println("Starting in 3 seconds...\n");
  delay(3000);
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  float duration = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT);
  float distance = duration ? (duration * 0.034) / 2 : MAX_DISTANCE;
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");
  return distance;
}

void carForward() {
  analogWrite(ENA, NORMAL_SPEED);
  analogWrite(ENB, NORMAL_SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Moving Forward");
}

void carBackward() {
  analogWrite(ENA, emergencyMode ? EMERGENCY_SPEED : BACKWARD_SPEED);
  analogWrite(ENB, emergencyMode ? EMERGENCY_SPEED : BACKWARD_SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Moving Backward");
}

void carTurnLeft() {
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Turning Left");
}

void carTurnRight() {
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Turning Right");
}

void carStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Stopped");
}

int scanForBestPath() {
  float leftDistance = 0, rightDistance = 0, centerDistance = 0;
  
  Serial.println("→ Scanning environment...");
  
  // Scan right
  sensorServo.write(20);
  delay(SCAN_DELAY);
  rightDistance = getDistance();
  
  // Scan left
  sensorServo.write(160);
  delay(SCAN_DELAY);
  leftDistance = getDistance();
  
  // Return to center
  sensorServo.write(90);
  delay(SCAN_DELAY);
  centerDistance = getDistance();
  
  Serial.print("→ Scan results - L:"); Serial.print(leftDistance);
  Serial.print(" C:"); Serial.print(centerDistance);
  Serial.print(" R:"); Serial.println(rightDistance);
  
  if (centerDistance > OBSTACLE_DISTANCE) {
    Serial.println("→ Center clear - forward");
    return 0;
  }
  if (rightDistance > leftDistance + 10 && rightDistance > SIDE_CLEAR_DISTANCE) {
    Serial.println("→ Right path selected");
    return 1;
  }
  if (leftDistance > SIDE_CLEAR_DISTANCE) {
    Serial.println("→ Left path selected");
    return -1;
  }
  if (rightDistance > TURN_DISTANCE) {
    Serial.println("→ Right acceptable");
    return 1;
  }
  Serial.println("→ All blocked - reverse needed");
  return 2;
}

void performTurn(int direction) {
  int turnTime = stuckCounter > 2 ? TURN_TIME_BASE * 1.5 : TURN_TIME_BASE;
  if (direction == 1) carTurnRight();
  else carTurnLeft();
  lastTurnDirection = direction;
  delay(turnTime);
  carStop();
  delay(200);
  stuckCounter = max(0, stuckCounter - 1);
}

void emergencyReverse() {
  Serial.println("⚠️ EMERGENCY MANEUVER!");
  emergencyMode = true;
  carStop();
  delay(200);
  carBackward();
  delay(500);
  carStop();
  delay(200);
  performTurn(lastTurnDirection == 1 ? -1 : 1);
  emergencyMode = false;
  stuckCounter++;
}

bool getSensorDataFromUnit2(float &moisture, float &temperature) {
  for (int attempts = 0; attempts < 3; attempts++) {
    WiFiClient client;
    if (!client.connect(serverIP, serverPort)) {
      Serial.println("Attempt " + String(attempts + 1) + ": Failed to connect to Unit 2");
      delay(1000);
      continue;
    }

    client.println("GET /sensordata HTTP/1.1");
    client.println("Host: " + String(serverIP));
    client.println("Connection: close");
    client.println();
    Serial.println("Attempt " + String(attempts + 1) + ": Sent /sensordata request");

    unsigned long timeout = millis() + HTTP_TIMEOUT;
    String response = "";
    while (client.connected() && millis() < timeout) {
      if (client.available()) {
        String line = client.readStringUntil('\n');
        if (line.startsWith("MOISTURE:") || line.startsWith("TEMP:")) {
          response += line + "\n";
        }
      }
      delay(10);
    }
    client.stop();

    if (response.length() == 0) {
      Serial.println("No response from Unit 2");
      continue;
    }
    
    int moistureIndex = response.indexOf("MOISTURE:");
    int tempIndex = response.indexOf("TEMP:");
    
    if (moistureIndex != -1 && tempIndex != -1) {
      String moistureLine = response.substring(moistureIndex);
      String tempLine = response.substring(tempIndex);
      moisture = moistureLine.substring(9, moistureLine.indexOf('\n')).toFloat();
      temperature = tempLine.substring(5, tempLine.indexOf('\n')).toFloat();
      Serial.println("✓ Data received: M=" + String(moisture) + "%, T=" + String(temperature) + "°C");
      return true;
    }
  }
  Serial.println("✗ Failed to get sensor data after 3 attempts");
  return false;
}

bool requestIrrigation() {
  WiFiClient client;
  if (!client.connect(serverIP, serverPort)) {
    Serial.println("✗ Failed to connect for irrigation");
    return false;
  }

  client.println("GET /irrigate HTTP/1.1");
  client.println("Host: " + String(serverIP));
  client.println("Connection: close");
  client.println();
  
  unsigned long timeout = millis() + HTTP_TIMEOUT;
  String response = "";
  while (client.connected() && millis() < timeout) {
    if (client.available()) {
      response += client.readStringUntil('\n');
    }
    delay(10);
  }
  client.stop();
  
  return response.indexOf("Irrigation completed") != -1;
}

void controlIrrigation() {
  Serial.println("\n╔════════════════════════════════╗");
  Serial.println("║  MOISTURE CHECK INITIATED     ║");
  Serial.println("╚════════════════════════════════╝");
  
  float moisture = -1;
  float temperature = -1;
  
  if (!getSensorDataFromUnit2(moisture, temperature)) {
    Serial.println("⚠️ Sensor read failed - waiting " + String(DATA_COLLECTION_WAIT) + "ms");
    delay(DATA_COLLECTION_WAIT);
    Serial.println("╚════════════════════════════════╝\n");
    return;
  }
  
  if (moisture < MOISTURE_THRESHOLD && temperature > TEMP_THRESHOLD) {
    Serial.println("✓ Irrigation conditions met:");
    Serial.println("  • Moisture: " + String(moisture) + "% < " + String(MOISTURE_THRESHOLD) + "%");
    Serial.println("  • Temperature: " + String(temperature) + "°C > " + String(TEMP_THRESHOLD) + "°C");
    Serial.println("Requesting irrigation...");
    
    if (requestIrrigation()) {
      Serial.println("✓ Irrigation completed");
      Serial.println("Waiting " + String(IRRIGATION_WAIT) + "ms");
      delay(IRRIGATION_WAIT);
    } else {
      Serial.println("✗ Irrigation failed");
      delay(DATA_COLLECTION_WAIT);
    }
  } else {
    Serial.println("✗ Irrigation not needed:");
    Serial.println("  • Moisture: " + String(moisture) + "% (need: < " + String(MOISTURE_THRESHOLD) + "%)");
    Serial.println("  • Temperature: " + String(temperature) + "°C (need: > " + String(TEMP_THRESHOLD) + "°C)");
    Serial.println("Waiting " + String(DATA_COLLECTION_WAIT) + "ms");
    delay(DATA_COLLECTION_WAIT);
    requestIrrigation(); // Return servo
  }
  
  Serial.println("╚════════════════════════════════╝\n");
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastLoopTime < LOOP_INTERVAL) return;
  lastLoopTime = currentMillis;

  // Periodic moisture check
  if (currentMillis - lastMoistureCheck >= MOISTURE_CHECK_INTERVAL) {
    carStop();
    delay(200); // Ensure car fully stopped
    controlIrrigation();
    lastMoistureCheck = currentMillis;
  }

  // Obstacle avoidance with ultrasonic servo
  distance = getDistance();
  
  if (distance > OBSTACLE_DISTANCE) {
    carForward();
    stuckCounter = max(0, stuckCounter - 1);
    if (emergencyMode && distance > OBSTACLE_DISTANCE + 20) {
      emergencyMode = false;
      Serial.println("Emergency mode cleared");
    }
  } else if (distance > EMERGENCY_DISTANCE) {
    carStop();
    delay(200);
    int path = scanForBestPath(); // This uses servo scanning
    switch (path) {
      case 0: carForward(); break;
      case 1: performTurn(1); break;
      case -1: performTurn(-1); break;
      case 2: emergencyReverse(); break;
    }
  } else {
    emergencyReverse();
  }
}
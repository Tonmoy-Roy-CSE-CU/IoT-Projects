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
const char* serverIP = "10.138.17.246"; // Unit 2 IP (verify this)
const int serverPort = 80;

// Constants
#define MAX_DISTANCE 200       // Max sensor distance (cm)
#define OBSTACLE_DISTANCE 35   // Stop if obstacle within 35cm
#define TURN_DISTANCE 45       // Scan if obstacle within 45cm
#define EMERGENCY_DISTANCE 20  // Emergency reverse if within 20cm
#define SIDE_CLEAR_DISTANCE 50 // Min clear distance for sides
#define NORMAL_SPEED 120       // Normal speed (0-255)
#define TURN_SPEED 140         // Turn speed
#define BACKWARD_SPEED 90      // Backward speed
#define EMERGENCY_SPEED 90     // Emergency speed
#define SCAN_DELAY 250         // Servo delay (ms)
#define TURN_TIME_BASE 800     // Base turn time (ms)
#define LOOP_INTERVAL 50       // Loop interval (ms)
#define PULSE_TIMEOUT 30000    // pulseIn timeout (us)
#define MOISTURE_CHECK_INTERVAL 30000 // Check moisture every 30s
#define MOISTURE_THRESHOLD 40  // Moisture % below which irrigation is requested
#define HTTP_TIMEOUT 5000      // HTTP request timeout (ms)

Servo sensorServo;
float distance;
int stuckCounter = 0;
int lastTurnDirection = 0; // 1 = right, -1 = left
unsigned long lastLoopTime = 0;
unsigned long lastMoistureCheck = 0;
bool emergencyMode = false;

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  sensorServo.attach(SERVO_PIN, 500, 2400);
  sensorServo.write(90); // Center servo
  carStop();
  delay(1000);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nUnit 1 WiFi connected: " + WiFi.localIP().toString());
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
  sensorServo.write(20); // Right
  delay(SCAN_DELAY);
  rightDistance = getDistance();
  sensorServo.write(160); // Left
  delay(SCAN_DELAY);
  leftDistance = getDistance();
  sensorServo.write(90); // Center
  delay(SCAN_DELAY);
  centerDistance = getDistance();
  
  Serial.print("Left: "); Serial.print(leftDistance);
  Serial.print(" Center: "); Serial.print(centerDistance);
  Serial.print(" Right: "); Serial.print(rightDistance); Serial.println(" cm");
  
  if (centerDistance > OBSTACLE_DISTANCE) return 0;
  if (rightDistance > leftDistance + 10 && rightDistance > SIDE_CLEAR_DISTANCE) return 1;
  if (leftDistance > SIDE_CLEAR_DISTANCE) return -1;
  if (rightDistance > TURN_DISTANCE) return 1;
  return 2; // Reverse
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

float getMoistureFromUnit2() {
  WiFiClient client;
  if (!client.connect(serverIP, serverPort)) {
    Serial.println("Failed to connect to Unit 2 at " + String(serverIP) + ":" + String(serverPort));
    return -1;
  }

  // Send HTTP GET request
  client.println("GET /moisture HTTP/1.1");
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

  Serial.println("Raw response from Unit 2: " + response); // Debug raw response
  
  if (response.startsWith("MOISTURE:")) {
    float moisture = response.substring(9).toFloat();
    Serial.println("Moisture from Unit 2: " + String(moisture) + "%");
    return moisture;
  } else {
    Serial.println("Invalid response format from Unit 2");
    return -1;
  }
}

void requestIrrigation() {
  WiFiClient client;
  if (!client.connect(serverIP, serverPort)) {
    Serial.println("Failed to connect to Unit 2 for irrigation request");
    return;
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
  
  Serial.println("Irrigation request response: " + response);
}

void controlIrrigation() {
  float moisture = getMoistureFromUnit2();
  if (moisture >= 0 && moisture < MOISTURE_THRESHOLD) {
    Serial.println("Moisture low - Requesting irrigation");
    requestIrrigation();
  } else if (moisture >= MOISTURE_THRESHOLD) {
    Serial.println("Moisture sufficient - No irrigation needed");
  }
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastLoopTime < LOOP_INTERVAL) return;
  lastLoopTime = currentMillis;

  // Periodic moisture check
  if (currentMillis - lastMoistureCheck >= MOISTURE_CHECK_INTERVAL) {
    carStop();
    controlIrrigation();
    lastMoistureCheck = currentMillis;
  }

  // Obstacle avoidance
  distance = getDistance();
  if (distance > OBSTACLE_DISTANCE) {
    carForward();
    stuckCounter = max(0, stuckCounter - 1);
    if (emergencyMode && distance > OBSTACLE_DISTANCE + 20) emergencyMode = false;
  } else if (distance > EMERGENCY_DISTANCE) {
    carStop();
    delay(200);
    int path = scanForBestPath();
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
/*
 * Robust Autonomous Obstacle Avoidance Robot for ESP8266 NodeMCU
 * Uses pulseIn for ultrasonic measurements and optimized navigation
 */

#include <Servo.h>

// Pin Definitions
#define TRIG_PIN D7    // GPIO13 - Ultrasonic Sensor TRIG (Safe)
#define ECHO_PIN D8    // GPIO15 - Ultrasonic Sensor ECHO (Safe with pull-down)
#define SERVO_PIN D0   // GPIO16 - Servo Motor (Non-PWM, safe for servo)
#define ENA D6         // GPIO12 - Motor A Enable (PWM-capable)
#define IN1 D1         // GPIO5  - Motor A Input 1 (Safe)
#define IN2 D2         // GPIO4  - Motor A Input 2 (Safe)
#define IN3 D3         // GPIO0  - Motor B Input 1 (Boot-sensitive, cautious)
#define IN4 D4         // GPIO2  - Motor B Input 2 (Boot-sensitive, cautious)
#define ENB D5         // GPIO14 - Motor B Enable (PWM-capable)

// Constants
#define MAX_DISTANCE 200       // Maximum sensor distance (cm)
#define OBSTACLE_DISTANCE 35   // Stop if obstacle within 35cm
#define TURN_DISTANCE 45       // Scan for path if obstacle within 45cm
#define EMERGENCY_DISTANCE 20  // Emergency reverse if obstacle within 20cm
#define SIDE_CLEAR_DISTANCE 50 // Minimum clear distance for side paths
#define NORMAL_SPEED 120       // Normal driving speed (0-1023)
#define TURN_SPEED 140          // Speed during turns
#define BACKWARD_SPEED 90      // Speed for non-emergency backward movement
#define EMERGENCY_SPEED 90     // Emergency reverse speed (reduced)
#define SCAN_DELAY 250         // Servo movement delay (ms)
#define TURN_TIME_BASE 800     // Base turn time (ms)
#define LOOP_INTERVAL 50       // Main loop interval (ms)
#define PULSE_TIMEOUT 30000    // pulseIn timeout (us, ~5m)

// Variables
Servo sensorServo;
float distance;
int stuckCounter = 0; // Counter for stuck situations
int lastTurnDirection = 0; // 1 = right, -1 = left
unsigned long lastLoopTime = 0;
bool emergencyMode = false;

void setup() {
  Serial.begin(9600);
  Serial.println("\n=== Robust Obstacle Avoidance Robot ===");
  
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Stop motors immediately to ensure boot safety
  carStop();
  
  // Initialize servo
  sensorServo.attach(SERVO_PIN, 500, 2400);
  sensorServo.write(90); // Center position
  delay(1000);
  
  // Test motor and servo movement
  Serial.println("Testing motors and servo...");
  testMotorAndServo();
  
  Serial.println("Robot initialized successfully!");
  Serial.println("Starting in 3 seconds...");
  delay(3000);
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  float duration_us = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT);
  float distance = duration_us ? (duration_us * 0.034) / 2 : MAX_DISTANCE;
  
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");
  return distance;
}

void testMotorAndServo() {
  // Test servo: Confirm 45° points right, 135° points left
  Serial.println("Testing servo...");
  sensorServo.write(20); // Right
  delay(500);
  Serial.println("Servo at 45° (should point right)");
  sensorServo.write(160); // Left
  delay(500);
  Serial.println("Servo at 135° (should point left)");
  sensorServo.write(90); // Center
  delay(500);
  Serial.println("Servo at 90° (center)");
  
  // Test motors: Confirm directions
  Serial.println("Testing motors...");
  carForward(); delay(1000); carStop(); delay(500);
  Serial.println("Tested forward");
  carBackward(); delay(1000); carStop(); delay(500);
  Serial.println("Tested backward");
  carTurnLeft(); delay(1000); carStop(); delay(500);
  Serial.println("Tested left turn");
  carTurnRight(); delay(1000); carStop(); delay(500);
  Serial.println("Tested right turn");
}

int scanForBestPath() {
  float leftDistance = 0, rightDistance = 0, centerDistance = 0;
  
  Serial.println("Scanning environment...");
  
  // Scan right (20 degrees)
  sensorServo.write(20);
  delay(SCAN_DELAY);
  rightDistance = getDistance();
  
  // Scan left (160 degrees)
  sensorServo.write(160);
  delay(SCAN_DELAY);
  leftDistance = getDistance();
  
  // Check center
  sensorServo.write(90);
  delay(SCAN_DELAY);
  centerDistance = getDistance();
  
  Serial.print("Distances - Left: "); Serial.print(leftDistance);
  Serial.print(" cm, Center: "); Serial.print(centerDistance);
  Serial.print(" cm, Right: "); Serial.print(rightDistance); Serial.println(" cm");
  
  if (centerDistance > OBSTACLE_DISTANCE) {
    Serial.println("Center path clear - continuing forward");
    return 0; // Go straight
  }
  
  if (rightDistance > leftDistance + 10 && rightDistance > SIDE_CLEAR_DISTANCE) {
    Serial.println("Right path selected");
    return 1; // Turn right
  } else if (leftDistance > SIDE_CLEAR_DISTANCE) {
    Serial.println("Left path selected");
    return -1; // Turn left
  } else if (rightDistance > TURN_DISTANCE) {
    Serial.println("Right path acceptable");
    return 1; // Turn right
  }
  
  Serial.println("All paths blocked - need to reverse");
  return 2; // Reverse needed
}

void carForward() {
  analogWrite(ENA, NORMAL_SPEED);
  analogWrite(ENB, NORMAL_SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); // Motor A forward (TEST: Confirm Motor A is left)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); // Motor B forward (TEST: Confirm Motor B is right)
  Serial.println("Moving Forward");
}

void carBackward() {
  analogWrite(ENA, emergencyMode ? EMERGENCY_SPEED : BACKWARD_SPEED);
  analogWrite(ENB, emergencyMode ? EMERGENCY_SPEED : BACKWARD_SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW); // Motor A backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); // Motor B backward
  Serial.println("Moving Backward");
}

void carTurnLeft() {
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); // Motor A forward (TEST: Confirm Motor A is left)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); // Motor B backward (TEST: Confirm Motor B is right)
  Serial.println("Turning Left");
}

void carTurnRight() {
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW); // Motor A backward (TEST: Confirm Motor A is left)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); // Motor B forward (TEST: Confirm Motor B is right)
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

void performTurn(int direction) {
  int turnTime = TURN_TIME_BASE;
  if (stuckCounter > 2) {
    turnTime *= 1.5; // Longer turns when stuck
  }
  
  if (direction == 1) {
    carTurnRight();
    lastTurnDirection = 1;
  } else {
    carTurnLeft();
    lastTurnDirection = -1;
  }
  
  delay(turnTime);
  carStop();
  delay(200);
  stuckCounter = max(0, stuckCounter - 1); // Reset stuck counter after turn
}

void emergencyReverse() {
  Serial.println("EMERGENCY MANEUVER!");
  emergencyMode = true;
  
  carStop();
  delay(200);
  
  carBackward();
  delay(500);
  carStop();
  delay(200);
  
  int emergencyDirection = (lastTurnDirection == 1) ? -1 : 1;
  performTurn(emergencyDirection);
  
  emergencyMode = false;
  stuckCounter++;
}

void handleStuckSituation() {
  stuckCounter++;
  Serial.print("Stuck situation #"); Serial.println(stuckCounter);
  
  if (stuckCounter > 5) {
    Serial.println("Severely stuck - performing 180 degree turn");
    carTurnRight();
    delay(TURN_TIME_BASE * 2);
    carStop();
    delay(500);
    stuckCounter = 0;
  }
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastLoopTime < LOOP_INTERVAL) {
    return; // Non-blocking loop interval
  }
  lastLoopTime = currentMillis;
  
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
    Serial.println("Obstacle detected - analyzing options...");
    delay(200);
    
    int pathDirection = scanForBestPath();
    
    switch (pathDirection) {
      case 0:
        carForward();
        break;
      case 1:
        performTurn(1);
        break;
      case -1:
        performTurn(-1);
        break;
      case 2:
        handleStuckSituation();
        emergencyReverse();
        break;
    }
  } else {
    emergencyReverse();
  }
}
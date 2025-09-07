/*
 * Autonomous Obstacle Avoidance Robot using ESP8266 NodeMCU
 * Uses ultrasonic sensor with servo scanning for intelligent navigation
 */

#include <Servo.h>

// Ultrasonic Sensor PINs
#define TRIG_PIN D7    // GPIO13 - connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN D8    // GPIO15 - connected to Ultrasonic Sensor's ECHO pin
#define SERVO_PIN D6   // GPIO12 - connected to Servo Motor (for sensor scanning)

// Motor PINs
#define ENA D0    // GPIO16 - Motor A Enable
#define IN1 D1    // GPIO5  - Motor A Input 1
#define IN2 D2    // GPIO4  - Motor A Input 2
#define IN3 D3    // GPIO0  - Motor B Input 1
#define IN4 D4    // GPIO2  - Motor B Input 2
#define ENB D5    // GPIO14 - Motor B Enable

// Distance thresholds
#define OBSTACLE_DISTANCE 30    // Stop if obstacle within 30cm
#define TURN_DISTANCE 40       // Start looking for path if obstacle within 40cm
#define EMERGENCY_DISTANCE 15  // Emergency reverse distance

// Robot speed
int Speed = 180;              // Motor speed (0-255)

// Sensor variables
float duration_us, distance_cm;
Servo sensorServo;            // Servo for scanning

void setup() {
  Serial.begin(115200);
  
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Initialize servo
  sensorServo.attach(SERVO_PIN,500,2400);
  sensorServo.write(90); // Center position
  delay(1000);
  
  // Initialize motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Stop motors initially
  carStop();
  
  Serial.println("Autonomous Obstacle Avoidance Robot Starting...");
  delay(2000);
  Serial.println("Robot Active!");
}

// Function to measure distance
float getDistance() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure echo pulse duration
  duration_us = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate distance (speed of sound = 343 m/s)
  distance_cm = (duration_us * 0.034) / 2;
  
  return distance_cm;
}

// Function to scan left and right for clear path
int scanForPath() {
  int leftDistance, rightDistance;
  
  Serial.println("Scanning for clear path...");
  
  // Scan right (30 degrees)
  sensorServo.write(30);
  delay(500);
  rightDistance = getDistance();
  
  // Scan left (150 degrees)  
  sensorServo.write(150);
  delay(500);
  leftDistance = getDistance();
  
  // Return to center
  sensorServo.write(90);
  delay(500);
  
  Serial.print("Left Distance: "); Serial.print(leftDistance); Serial.println(" cm");
  Serial.print("Right Distance: "); Serial.print(rightDistance); Serial.println(" cm");
  
  // Return direction: 1 = right, -1 = left, 0 = both blocked
  if (rightDistance > leftDistance && rightDistance > TURN_DISTANCE) {
    Serial.println("Right path is clearer");
    return 1; // Turn right
  } else if (leftDistance > TURN_DISTANCE) {
    Serial.println("Left path is clearer");
    return -1; // Turn left
  }
  Serial.println("Both sides blocked");
  return 0; // Both sides blocked
}

// Motor control functions
void carForward() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // Motor A forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);   // Motor B forward
}

void carBackward() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);   // Motor A backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  // Motor B backward
}

void carTurnLeft() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // Motor A forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  // Motor B backward
}

void carTurnRight() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);   // Motor A backward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);   // Motor B forward
}

void carStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  // Get front distance
  float frontDistance = getDistance();
  
  Serial.print("Front Distance: "); Serial.print(frontDistance); Serial.println(" cm");
  
  if (frontDistance > OBSTACLE_DISTANCE) {
    // Path is clear, move forward
    carForward();
    Serial.println("Moving Forward");
    delay(100);
  } 
  else if (frontDistance > EMERGENCY_DISTANCE) {
    // Obstacle detected, stop and scan for alternative path
    carStop();
    Serial.println("Obstacle detected! Stopping...");
    delay(300);
    
    int direction = scanForPath();
    
    if (direction == 1) {
      // Turn right
      Serial.println("Turning Right");
      carTurnRight();
      delay(800); // Turn duration
      carStop();
      delay(200);
    } 
    else if (direction == -1) {
      // Turn left  
      Serial.println("Turning Left");
      carTurnLeft();
      delay(800); // Turn duration
      carStop();
      delay(200);
    } 
    else {
      // Both sides blocked, reverse and turn
      Serial.println("Dead end! Reversing and turning...");
      carBackward();
      delay(1000);
      carStop();
      delay(300);
      
      // Try turning right after reversing
      carTurnRight();
      delay(1000);
      carStop();
      delay(300);
    }
  }
  else {
    // Very close obstacle, emergency maneuver
    Serial.println("EMERGENCY! Object too close - Reversing!");
    carStop();
    delay(200);
    carBackward();
    delay(1500);
    carStop();
    delay(500);
    
    // Turn right after emergency reverse
    carTurnRight();
    delay(1200);
    carStop();
    delay(300);
  }
  
  delay(50); // Small delay for stability
}
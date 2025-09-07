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

Servo servo;

// Constants
const int OBSTACLE_DISTANCE = 30; // Distance threshold in cm
const int MOTOR_SPEED = 150;      // PWM speed (0-255)
const int SCAN_DELAY = 200;       // Delay for servo movement

void setup() {
  // Initialize pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Attach servo with specific pulse widths
  servo.attach(SERVO_PIN, 500, 2400);
  servo.write(90); // Center the servo

  // Start Serial for debugging
  Serial.begin(9600);
}

float getDistance() {
  // Send ultrasonic pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure echo time
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout after 30ms
  // Calculate distance in cm
  float distance = duration ? 0.017 * duration : 1000; // Return large value if no echo

  return distance;
}

void moveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // Motor A forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);   // Motor B forward
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}

void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);   // Motor A backward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);   // Motor B forward
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // Motor A forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  // Motor B backward
  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void loop() {
  // Look forward
  servo.write(90);
  delay(SCAN_DELAY);
  float frontDistance = getDistance();
  Serial.print("Front Distance: ");
  Serial.print(frontDistance);
  Serial.println(" cm");

  if (frontDistance < OBSTACLE_DISTANCE && frontDistance > 0) {
    stopMotors();
    delay(200);

    // Scan left (180 degrees)
    servo.write(180);
    delay(SCAN_DELAY);
    float leftDistance = getDistance();
    Serial.print("Left Distance: ");
    Serial.print(leftDistance);
    Serial.println(" cm");

    // Scan right (0 degrees)
    servo.write(0);
    delay(SCAN_DELAY);
    float rightDistance = getDistance();
    Serial.print("Right Distance: ");
    Serial.print(rightDistance);
    Serial.println(" cm");

    // Return servo to center
    servo.write(90);
    delay(SCAN_DELAY);

    // Decide which way to turn
    if (leftDistance > rightDistance && leftDistance > OBSTACLE_DISTANCE) {
      turnLeft();
      delay(500); // Turn for a short duration
    } else if (rightDistance > OBSTACLE_DISTANCE) {
      turnRight();
      delay(500); // Turn for a short duration
    } else {
      // If both directions are blocked, turn right by default
      turnRight();
      delay(1000); // Turn longer to try escaping
    }
  } else {
    moveForward();
  }
}
#define BLYNK_TEMPLATE_ID "TMPL6aG_9BmYK"
#define BLYNK_TEMPLATE_NAME "Fire Fighter Robo m1"
#define BLYNK_AUTH_TOKEN "X6LR7zHDQ6Qf1IC49_19UiugkeuaWyLH"

#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

// Motor PINs
#define ENA D0    // GPIO16 (no PWM; consider D6 if PWM needed)
#define IN1 D1    // GPIO5
#define IN2 D2    // GPIO4
#define IN3 D3    // GPIO0
#define IN4 D4    // GPIO2
#define ENB D5    // GPIO14 (PWM-capable)

bool forward = false;
bool backward = false;
bool left = false;
bool right = false;
int Speed = 200; // Default speed

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "BOT-123";
char pass[] = "12345123";

void setup() {
  Serial.begin(115200); // Higher baud rate for better Serial Monitor performance

  // Initialize motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  Blynk.begin(auth, ssid, pass);
}

// Blynk button controls
BLYNK_WRITE(V0) { forward = param.asInt() != 0; }
BLYNK_WRITE(V1) { backward = param.asInt() != 0; }
BLYNK_WRITE(V2) { left = param.asInt() != 0; }
BLYNK_WRITE(V3) { right = param.asInt() != 0; }
BLYNK_WRITE(V4) { Speed = param.asInt(); }

void smartcar() {
  if (forward) {
    carForward();
    Serial.println("Car Forward");
  } else if (backward) {
    carBackward();
    Serial.println("Car Backward");
  } else if (left) {
    carTurnLeft();
    Serial.println("Car Turn Left");
  } else if (right) {
    carTurnRight();
    Serial.println("Car Turn Right");
  } else {
    carStop();
    Serial.println("Car Stop");
  }
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
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);   // Motor A backward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);   // Motor B forward
}

void carTurnRight() {
  analogWrite(ENA, Speed);
  analogWrite(ENB, Speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // Motor A forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  // Motor B backward
}

void carStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  Blynk.run();
  smartcar();
}
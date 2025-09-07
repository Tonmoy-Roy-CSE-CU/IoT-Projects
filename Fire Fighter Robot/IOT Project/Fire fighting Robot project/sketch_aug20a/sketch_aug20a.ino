#define BLYNK_TEMPLATE_ID "TMPL63LObusnV"
#define BLYNK_TEMPLATE_NAME "Fire Fighting Robot"
#define BLYNK_AUTH_TOKEN "OFTn72WiMiV-ZHPLURGHZLn0TTgqIMDA"

#define BLYNK_PRINT Serial
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
//Motor PINs
#define ENA D0
#define IN1 D1
#define IN2 D2
#define IN3 D3
#define IN4 D4
#define ENB D5

Servo motor1;
int F_sensor=12;//D6
int pump=13;//buzzer also D7
int servopin=15;//D8
int green_led=1;//TX
int red_led=3;//RX
bool forward = 0;
bool backward = 0;
bool left = 0;
bool right = 0;
int Speed;
char auth[] = BLYNK_AUTH_TOKEN; //Enter your Blynk application auth token
char ssid[] = "নিশাচর"; //Enter your WIFI name
char pass[] = "alamin2023"; //Enter your WIFI passowrd
void setup() {
//Initialize the serial monitor
 Serial.begin(115200);
//Set the motor pins as the output pin
 pinMode(ENA, OUTPUT);
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);
 pinMode(ENB, OUTPUT);
 
 motor1.attach(servopin);
 pinMode(pump, OUTPUT);
 pinMode(green_led, OUTPUT);
 pinMode(red_led, OUTPUT);
 digitalWrite(pump,HIGH);
//Initialize the blynk communication
 Blynk.begin(auth, ssid, pass);
}

//Motor control functions
void Forward() {
 analogWrite(ENA, Speed);
 analogWrite(ENB, Speed);
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, HIGH);
 digitalWrite(IN3, HIGH);
 digitalWrite(IN4, LOW);
}
void Backward() {
 analogWrite(ENA, Speed);
 analogWrite(ENB, Speed);
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, LOW);
 digitalWrite(IN4, HIGH);
}
void Left() {
 analogWrite(ENA, Speed);
 analogWrite(ENB, Speed);
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, HIGH);
 digitalWrite(IN4, LOW);
}
void Right() {
 analogWrite(ENA, Speed);
 analogWrite(ENB, Speed);
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, HIGH);
 digitalWrite(IN3, LOW);
 digitalWrite(IN4, HIGH);
}
void Stop() {
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, LOW);
 digitalWrite(IN4, LOW);
}

void put_off_fire() {
  
  delay (200); 
  Stop();
  delay(100);
  
  digitalWrite(pump, LOW);
  //delay(500);
  int pos;
  for (pos = 50; pos <= 130; pos += 1) {
      motor1.write(pos);
      delay(10); 
  }
  for (pos = 130; pos >= 50; pos -= 1) {
      motor1.write(pos);
      delay(10);
  }
  digitalWrite(pump,HIGH);
  motor1.write(90);
}
void manualWATER() {
//you begin your own personal code for servo here
 int pos;
 digitalWrite(pump,LOW);
 for (pos = 50; pos <= 140; pos ++) { // goes from 40 degrees to 140 degrees
 // in steps of 1 degree
 motor1.write(pos); // tell servo to go to position in variable 'pos'
 delay(10); // waits 15ms for the servo to reach the position
 }
 for (pos = 140; pos >=50; pos --) { // goes from 140 degrees to 40 degrees
  motor1.write(pos); // tell servo to go to position in variable 'pos'
  delay(10); // waits 15ms for the servo to reach the position
 }
  digitalWrite(pump,HIGH);
  delay(10);
 
}


//Get values from the widgets
BLYNK_WRITE(V0) {
 forward = param.asInt();
}
BLYNK_WRITE(V1) {
 backward = param.asInt();
}
BLYNK_WRITE(V2) {
 left = param.asInt();
}
BLYNK_WRITE(V3) {
 right = param.asInt();
}
BLYNK_WRITE(V4) {
 Speed = param.asInt();
}
//Extinguish fire
BLYNK_WRITE(V5) {
  int pinValue = param.asInt();
  if (pinValue == 1) { // if Button sends 1
    Serial.println("servo and led is on");
    digitalWrite(red_led,HIGH);
    digitalWrite(green_led,LOW);
    manualWATER(); // start the function cradle
    Blynk.run(); // Run rest of show in-between waiting for this loop to repeat or quit.
    int pinValue = 0; // Set V5 status to 0 to quit, unless button is still pushed (as per below)
    Blynk.syncVirtual(V5); // ...Then force BLYNK_WRITE(V0) function check of button status to determine if repeating or done.
  }
 else{
    digitalWrite(red_led,LOW);
    digitalWrite(green_led,HIGH);
    digitalWrite(pump,HIGH);
    Serial.println("servo and led is off");
  }
}
//Autonomous mode
BLYNK_WRITE(V6) 
{
 int pinValue = param.asInt();
 if (pinValue == 1) { // if Button sends 1
    if (digitalRead(F_sensor) ==0) //If Fire not detected all sensors are zero
        {
        //Do not move the robot
        digitalWrite(green_led,HIGH);
        digitalWrite(red_led,LOW);
        Stop();
      } 
    else if (digitalRead(F_sensor) ==1) //If Fire is straight ahead
    {
        //Move the robot forward
        Forward();
        digitalWrite(green_led,LOW);
        digitalWrite(red_led,HIGH);
        Blynk.logEvent("fire_fighting_robot","Fire Fire Fire...!!! Be alert soon.......!");
        
    }
    delay(300); //Slow down the speed of robot and change this value to increase the distance 
    while(digitalRead(F_sensor)){
      digitalWrite(red_led,HIGH);
      digitalWrite(green_led,LOW);
      Blynk.logEvent("fire_fighting_robot","Fire Fire Fire...!!! Be alert soon.......!");
      put_off_fire();
    } 
    Blynk.run(); // Run rest of show in-between waiting for this loop to repeat or quit.
    int pinValue = 0; // Set V6 status to 0 to quit, unless button is still pushed (as per below)
    Blynk.syncVirtual(V6);
 
 }
 else{
 Serial.println("AUTOMATIC MODE is off");
 }
}
//Check widget values using the IF condition
void smartcar() {
 if (forward == 1) {
 Forward();
 Serial.println("Forward");
 } else if (backward == 1) {
 Backward();
 Serial.println("Backward");
 } else if (left == 1) {
 Left();
 Serial.println("Left");
 } else if (right == 1) {
 Right();
 Serial.println("Right");
 } else if (forward == 0 && backward == 0 && left == 0 && right == 0) {
 Stop();
 Serial.println("Stop");
 }
}
void loop() {
//Run the blynk library
 Blynk.run();
 smartcar();
 if(digitalRead(F_sensor)==0){
    digitalWrite(green_led,HIGH);
    digitalWrite(red_led,LOW);
 }
 else if(digitalRead(F_sensor)==1){
   digitalWrite(green_led,LOW);
   digitalWrite(red_led,HIGH);
 }
}


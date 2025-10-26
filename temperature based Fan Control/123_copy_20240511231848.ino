#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
\video\yH34TCThm5k\edit
#define DHTPIN 3
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

const int potPin = A0;
const int RelayPin = 5; // Connect Relay to this pin

LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address and dimensions

void setup() {
  dht.begin();
  pinMode(RelayPin, OUTPUT);
  lcd.begin();                      // Initialize the LCD
  lcd.backlight();                 // Turn on the backlight
  lcd.setCursor(0, 0);
  lcd.print("Temp Fan Control");
  lcd.setCursor(0, 1);
  lcd.print("by Your Name");
  delay(2000);
  lcd.clear();
}

void loop() {
  int threshold = map(analogRead(potPin), 0, 1023, 20, 40); // Map potentiometer value to temperature range

  float temperature = dht.readTemperature();

  if (temperature > threshold) {
    digitalWrite(RelayPin, HIGH); // Turn on the fan
  } else {
    digitalWrite(RelayPin, LOW); // Turn off the fan
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("C");

  lcd.setCursor(0, 1);
  lcd.print("SetTemp: ");
  lcd.print(threshold);
  lcd.print("C");

  delay(1000);
}

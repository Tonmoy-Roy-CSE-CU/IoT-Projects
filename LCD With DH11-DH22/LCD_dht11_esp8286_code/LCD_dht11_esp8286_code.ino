#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

#define DHTPIN 13        // GPIO13 (D7) - DHT22 data pin
#define DHTTYPE DHT11    // DHT22 sensor type
#define LEDPIN 14        // GPIO14 (D5) for LED

float tempThreshold = 25.0;  // LED turns ON above this temperature

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Changed to 16x2 LCD

void setup() {
  Serial.begin(9600);
  dht.begin();
  pinMode(LEDPIN, OUTPUT);

  Wire.begin();  // Default I2C pins: SDA (GPIO4/D2), SCL (GPIO5/D1)

  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);  // Adjusted for 16x2
  lcd.print("DHT22 TEST");
  delay(2000);
  lcd.clear();
}

void loop() {
  delay(2000); // Wait between readings

  float humidity = dht.readHumidity();
  float temperatureC = dht.readTemperature();
  float temperatureF = dht.readTemperature(true);

  // Check if readings failed
  if (isnan(humidity) || isnan(temperatureC) || isnan(temperatureF)) {
    Serial.println("❌ Sensor Error!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sensor Error!");
    return;
  }

  // Serial Output (unchanged)
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.print(" °C  ~  ");
  Serial.print(temperatureF);
  Serial.println(" °F");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  // LCD Output - Adjusted for 16x2
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperatureC, 1);
  lcd.print("C / ");
  lcd.print(temperatureF, 1);
  lcd.print("F");

  lcd.setCursor(0, 1);
  lcd.print("H:");
  lcd.print(humidity, 1);
  lcd.print("% ");

  // LED control and status (abbreviated)
  if (temperatureC > tempThreshold) {
    digitalWrite(LEDPIN, HIGH);
    lcd.print("LED:ON");
  } else {
    digitalWrite(LEDPIN, LOW);
    lcd.print("LED:OFF");
  }
}
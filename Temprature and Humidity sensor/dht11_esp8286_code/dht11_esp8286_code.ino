#include <DHT.h>

#define DHTPIN 13       // ESP32 pin connected to DHT sensor data
#define DHTTYPE DHT11   // Change to DHT11 if using DHT11

#define LEDPIN 5        // GPIO5 connected to LED

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();           // Initialize DHT sensor
  pinMode(LEDPIN, OUTPUT); // Initialize LED pin
}

void loop() {
  delay(2000); // Wait between readings

  float humidity = dht.readHumidity();
  float temperatureC = dht.readTemperature();
  float temperatureF = dht.readTemperature(true);

  if (isnan(humidity) || isnan(temperatureC) || isnan(temperatureF)) {
    Serial.println("❌ Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.print(" °C  ~  ");
  Serial.print(temperatureF);
  Serial.println(" °F");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  // Condition to control LED
  if (temperatureC > 25) {
    digitalWrite(LEDPIN, LOW); // Turn LED on
    Serial.println("💡 LED ON - Temperature above 25°C");
  } else {
    digitalWrite(LEDPIN, LOW);  // Turn LED off
    Serial.println("💡 LED OFF - Temperature below 25°C");
  }
}

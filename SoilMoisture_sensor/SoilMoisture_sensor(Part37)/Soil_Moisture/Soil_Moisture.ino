// YouTube | Tech at Home

int sensor_pin = A0;

void setup()
{
  Serial.begin(9600);
  pinMode(sensor_pin, INPUT);
}

void loop()
{
  int sensor_data = analogRead(sensor_pin);
  Serial.print("Sensor_data: ");
  Serial.print(sensor_data);
  Serial.print("\t | ");

  if (sensor_data >= 950 && sensor_data <= 1024) // No moisture
  {
    Serial.println("No moisture, Soil is dry (Level 0) - Irrigate!");
  }
  else if (sensor_data >= 875 && sensor_data < 950) // Level 1
  {
    Serial.println("Level 1: Very dry soil - Irrigate!");
  }
  else if (sensor_data >= 840 && sensor_data < 875) // Level 2
  {
    Serial.println("Level 2: Dry soil - Irrigate!");
  }
  else if (sensor_data >= 810 && sensor_data < 840) // Level 3
  {
    Serial.println("Level 3: Slightly dry soil - Consider irrigation");
  }
  else if (sensor_data >= 750 && sensor_data < 810) // Level 4
  {
    Serial.println("Level 4: Medium dry soil - Within standard range");
  }
  else if (sensor_data >= 691 && sensor_data < 750) // Level 5
  {
    Serial.println("Level 5: Slightly moist soil - Within standard range");
  }
  else if (sensor_data >= 680 && sensor_data < 691) // Level 6
  {
    Serial.println("Level 6: Moist soil - Within standard range");
  }
  else if (sensor_data >= 640 && sensor_data < 680) // Level 7
  {
    Serial.println("Level 7: Very moist soil - Monitor for waterlogging");
  }
  else if (sensor_data < 640 && sensor_data >= 600) // Below Level 7 to min
  {
    Serial.println("Soil is wet (below Level 7) - Risk of waterlogging");
  }
  else if (sensor_data < 600) // Wettest
  {
    Serial.println("Soil is fully saturated - Drain if possible");
  }

  delay(100);
}
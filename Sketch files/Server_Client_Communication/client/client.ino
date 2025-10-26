#include <ESP8266WiFi.h>

const char* ssid = "BOT-123";
const char* password = "12345123";
const char* serverIP = "10.138.17.246"; // Replace with ESP8266 Server's IP
const int serverPort = 80;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void loop() {
  WiFiClient client;
  if (client.connect(serverIP, serverPort)) {
    client.println("Hello from ESP8266 Client");
    Serial.println("Sent: Hello from ESP8266 Client");
    
    String response = client.readStringUntil('\n');
    Serial.println("Server response: " + response);
    
    client.stop();
  } else {
    Serial.println("Connection failed");
  }
  delay(5000); // Send every 5 seconds
}
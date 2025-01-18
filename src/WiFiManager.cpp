#include "WiFiManager.h"

WiFiManager::WiFiManager(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);
}

void WiFiManager::connect() {
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

IPAddress WiFiManager::getLocalIP() {
  return WiFi.localIP();
}

bool WiFiManager::isConnected() {
  return WiFi.status() == WL_CONNECTED;
}

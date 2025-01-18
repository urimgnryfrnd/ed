#include <WiFi.h>
#include <WebServer.h>
#include "secrets.h"  // Make sure this file defines WIFI_SSID and WIFI_PASSWORD
#include "MotorController.h"
#include "WiFiManager.h"
#include "HttpHandlers.h"

// ============================= Web Server =============================
WebServer server(80);

// Instantiate MotorController
MotorController motorController;

// Instantiate WiFiManager
WiFiManager wifiManager(WIFI_SSID, WIFI_PASSWORD);

// Instantiate HttpHandlers
HttpHandlers httpHandlers(server, motorController);

// ========================= SETUP =========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Mecanum Wheeled Robot - Modular Sketch");
  
  // Initialize Motor Pins and PWM
  motorController.initializeMotors();
  motorController.initializePWM();

  // Connect to WiFi
  wifiManager.connect();

  // Set up HTTP routes
  httpHandlers.setupRoutes();

  // Start the web server
  server.begin();
  Serial.println("Web server started");
}

// ========================= LOOP =========================
void loop() {
  // Handle incoming HTTP requests
  server.handleClient();
}

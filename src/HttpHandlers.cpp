#include "HttpHandlers.h"

// Include any additional libraries if needed

HttpHandlers::HttpHandlers(WebServer& srv, MotorController& motorCtrl)
  : server(srv), motorController(motorCtrl) {}

void HttpHandlers::setupRoutes() {
  server.on("/",        [this]() { handleRoot(); });
  server.on("/move",    [this]() { handleMove(); });
  server.on("/emotion", [this]() { handleEmotion(); });
  server.on("/set_pwm", [this]() { handleSetPWM(); });
  server.on("/get_pwm", [this]() { handleGetPWM(); });
}

void HttpHandlers::handleRoot() {
  // You can serve the HTML directly or serve it from SPIFFS/LittleFS
  // For simplicity, I'll keep it as a string (similar to your original code)
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <!-- Your existing HTML content here -->
  <!-- For brevity, it's omitted. You can include it as a separate file and read from SPIFFS -->
  </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void HttpHandlers::handleMove() {
  if (server.hasArg("direction")) {
    String direction = server.arg("direction");

    if (direction == "circularRadius") {
      if (server.hasArg("linear") && server.hasArg("radius")) {
        int linear = server.arg("linear").toInt();
        float radius = server.arg("radius").toFloat();
        motorController.moveCircularWithRadius(linear, radius);
        String resp = "Moving in Circular Path with Linear=" + String(linear) +
                      " and Radius=" + String(radius);
        server.send(200, "text/plain", resp);
      } else {
        server.send(400, "text/plain", "Missing 'linear' or 'radius' parameter");
      }
    }
    else if (direction == "circularCenterFacing") {
      if (server.hasArg("linear") && server.hasArg("radius")) {
        int linear = server.arg("linear").toInt();
        float radius = server.arg("radius").toFloat();
        motorController.moveCircularCenterFacing(linear, radius);
        String resp = "Center-Facing Circular with Linear=" + String(linear) +
                      " and Radius=" + String(radius);
        server.send(200, "text/plain", resp);
      } else {
        server.send(400, "text/plain", "Missing 'linear' or 'radius' parameter");
      }
    }
    else if (direction == "square") {
      if (server.hasArg("duration")) {
        int duration = server.arg("duration").toInt();
        if (duration <= 0) {
          server.send(400, "text/plain", "Invalid 'duration' parameter");
          return;
        }
        motorController.moveSquare(duration);
        String resp = "Executing Square Trajectory for " + String(duration) + " seconds per side.";
        server.send(200, "text/plain", resp);
      } else {
        server.send(400, "text/plain", "Missing 'duration' parameter");
      }
    }
    else {
      motorController.moveRobot(direction);
      server.send(200, "text/plain", "Moving: " + direction);
    }
  } else {
    server.send(400, "text/plain", "Bad Request: Missing 'direction' parameter");
  }
}

void HttpHandlers::handleEmotion() {
  if (server.hasArg("plain")) {
    String direction = server.arg("plain");
    motorController.moveRobot(direction);
    server.send(200, "text/plain", "Moving (emotion-based): " + direction);
  } else {
    server.send(400, "text/plain", "Bad Request: Missing 'direction' parameter");
  }
}

void HttpHandlers::handleSetPWM() {
  if (server.hasArg("motor") && server.hasArg("pwm")) {
    String motor = server.arg("motor");
    int pwmVal = server.arg("pwm").toInt();
    motorController.setPWM(motor, pwmVal);
    String response = "Set " + motor + " PWM to " + String(pwmVal);
    server.send(200, "text/plain", response);
  } else {
    server.send(400, "text/plain", "Missing 'motor' or 'pwm' parameter");
  }
}

void HttpHandlers::handleGetPWM() {
  // Prepare JSON response
  DynamicJsonDocument doc(1024);
  uint16_t pwmValues[8];
  motorController.getAllPWM(pwmValues);
  String motorNames[] = {"Motor1_RPWM", "Motor1_LPWM", "Motor2_RPWM", "Motor2_LPWM",
                        "Motor3_RPWM", "Motor3_LPWM", "Motor4_RPWM", "Motor4_LPWM"};
  for(int i = 0; i < 8; i++) {
    doc[motorNames[i]] = pwmValues[i];
  }
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

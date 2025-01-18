#ifndef HTTP_HANDLERS_H
#define HTTP_HANDLERS_H

#include <WebServer.h>
#include "MotorController.h"
#include <ArduinoJson.h>

class HttpHandlers {
public:
  HttpHandlers(WebServer& server, MotorController& motorController);
  void setupRoutes();

private:
  WebServer& server;
  MotorController& motorController;
  
  // Handler functions
  void handleRoot();
  void handleMove();
  void handleEmotion();
  void handleSetPWM();
  void handleGetPWM();
};

#endif // HTTP_HANDLERS_H

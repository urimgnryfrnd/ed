#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

struct MotorPWM {
  String name;   // e.g., "Motor1_RPWM"
  int channel;   // channel number for ledcWrite
  int value;     // current PWM value (0–1023)
};

class MotorController {
public:
  MotorController();
  void initializeMotors();
  void initializePWM();
  
  // Basic Movements
  void moveForward(int speed);
  void moveBackward(int speed);
  void strafeLeft(int speed);
  void strafeRight(int speed);
  void rotateLeft(int speed);
  void rotateRight(int speed);
  
  // Diagonal Movements
  void moveDiagonalForwardLeft(int speed);
  void moveDiagonalForwardRight(int speed);
  void moveDiagonalBackwardLeft(int speed);
  void moveDiagonalBackwardRight(int speed);
  
  // Circular Motions
  void moveCircular(int translationalSpeed, int rotationalSpeed);
  void moveCircularWithRadius(int linearPWM, float radius);
  void moveCircularCenterFacing(int linearPWM, float radius);
  
  // Square Trajectory
  void moveSquare(int durationSeconds);
  
  // Robot Control
  void stopRobot();
  void moveRobot(String direction); // Ensure this line exists and is public
  
  // PWM Management
  void setPWM(String motor, int pwmVal);
  int getPWM(String motor);
  void getAllPWM(uint16_t* pwmValues);

private:
  static const int NUM_MOTORS = 8;
  MotorPWM motorPWMs[NUM_MOTORS];
  
  void updatePWM(String motor, int pwmVal);
};

#endif // MOTOR_CONTROLLER_H

#include "MotorController.h"

// Motor Pin Definitions
const int MOTOR1_RPWM = 26; // GPIO26
const int MOTOR1_LPWM = 25; // GPIO25

const int MOTOR2_RPWM = 15; // GPIO15
const int MOTOR2_LPWM = 4;  // GPIO4

const int MOTOR3_RPWM = 12; // GPIO12
const int MOTOR3_LPWM = 13; // GPIO13

const int MOTOR4_RPWM = 19; // GPIO19
const int MOTOR4_LPWM = 18; // GPIO18

// PWM Configuration
const int PWM_FREQ       = 1000; // PWM frequency in Hz
const int PWM_RESOLUTION = 10;   // 10-bit resolution (0-1023)
const int PWM_CHANNELS   = 10;   // We'll use channels 0..9 as needed

MotorController::MotorController() {
  // Initialize motorPWMs array
  MotorPWM temp[] = {
    {"Motor1_RPWM", 6, 0},
    {"Motor1_LPWM", 7, 0},
    {"Motor2_RPWM", 2, 0},
    {"Motor2_LPWM", 3, 0},
    {"Motor3_RPWM", 8, 0},
    {"Motor3_LPWM", 9, 0},
    {"Motor4_RPWM", 4, 0},
    {"Motor4_LPWM", 5, 0}
  };
  
  for(int i = 0; i < NUM_MOTORS; i++) {
    motorPWMs[i] = temp[i];
  }
}

void MotorController::initializeMotors() {
  // Set all motor pins as OUTPUT
  pinMode(MOTOR1_RPWM, OUTPUT);
  pinMode(MOTOR1_LPWM, OUTPUT);
  pinMode(MOTOR2_RPWM, OUTPUT);
  pinMode(MOTOR2_LPWM, OUTPUT);
  pinMode(MOTOR3_RPWM, OUTPUT);
  pinMode(MOTOR3_LPWM, OUTPUT);
  pinMode(MOTOR4_RPWM, OUTPUT);
  pinMode(MOTOR4_LPWM, OUTPUT);

  // Ensure motors start in stopped state
  stopRobot();
}

void MotorController::initializePWM() {
  for(int i = 0; i < NUM_MOTORS; i++) {
    ledcSetup(motorPWMs[i].channel, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(
      (motorPWMs[i].name == "Motor1_RPWM") ? MOTOR1_RPWM :
      (motorPWMs[i].name == "Motor1_LPWM") ? MOTOR1_LPWM :
      (motorPWMs[i].name == "Motor2_RPWM") ? MOTOR2_RPWM :
      (motorPWMs[i].name == "Motor2_LPWM") ? MOTOR2_LPWM :
      (motorPWMs[i].name == "Motor3_RPWM") ? MOTOR3_RPWM :
      (motorPWMs[i].name == "Motor3_LPWM") ? MOTOR3_LPWM :
      (motorPWMs[i].name == "Motor4_RPWM") ? MOTOR4_RPWM :
      MOTOR4_LPWM,
      motorPWMs[i].channel
    );
  }
  Serial.println("PWM Initialized");
}

void MotorController::stopRobot() {
  for(int i = 0; i < PWM_CHANNELS; i++) {
    ledcWrite(i, 0);
  }
  for(int i = 0; i < NUM_MOTORS; i++) {
    motorPWMs[i].value = 0;
  }
  Serial.println("Stopping Robot");
}

void MotorController::moveForward(int speed) {
  speed = constrain(speed, 0, 1023);

  // All wheels forward
  ledcWrite(6, speed); // M1_RPWM
  ledcWrite(7, 0);     
  ledcWrite(2, speed); // M2_RPWM
  ledcWrite(3, 0);
  ledcWrite(8, speed); // M3_RPWM
  ledcWrite(9, 0);
  ledcWrite(4, speed); // M4_RPWM
  ledcWrite(5, 0);

  Serial.println("Moving Forward");
}

void MotorController::moveBackward(int speed) {
  speed = constrain(speed, 0, 1023);

  // All wheels backward
  ledcWrite(6, 0);
  ledcWrite(7, speed);
  ledcWrite(2, 0);
  ledcWrite(3, speed);
  ledcWrite(8, 0);
  ledcWrite(9, speed);
  ledcWrite(4, 0);
  ledcWrite(5, speed);

  Serial.println("Moving Backward");
}

void MotorController::strafeLeft(int speed) {
  speed = constrain(speed, 0, 1023);

  // Mecanum Strafe Left
  ledcWrite(6, speed); // Motor1_RPWM
  ledcWrite(7, 0);     // Motor1_LPWM

  ledcWrite(2, speed); // Motor2_RPWM
  ledcWrite(3, 0);     // Motor2_LPWM

  ledcWrite(8, 0);     // Motor3_RPWM
  ledcWrite(9, speed); // Motor3_LPWM

  ledcWrite(4, 0);     // Motor4_RPWM
  ledcWrite(5, speed); // Motor4_LPWM

  Serial.println("Strafing Left");
}

void MotorController::strafeRight(int speed) {
  speed = constrain(speed, 0, 1023);

  // Mecanum Strafe Right
  ledcWrite(6, 0);     // Motor1_RPWM
  ledcWrite(7, speed); // Motor1_LPWM

  ledcWrite(2, 0);     // Motor2_RPWM
  ledcWrite(3, speed); // Motor2_LPWM

  ledcWrite(8, speed); // Motor3_RPWM
  ledcWrite(9, 0);     // Motor3_LPWM

  ledcWrite(4, speed); // Motor4_RPWM
  ledcWrite(5, 0);     // Motor4_LPWM

  Serial.println("Strafing Right");
}

void MotorController::rotateLeft(int speed) {
  speed = constrain(speed, 0, 1023);

  // Rotate Left
  ledcWrite(6, 0);
  ledcWrite(7, speed); 
  ledcWrite(2, speed);
  ledcWrite(3, 0);
  ledcWrite(8, speed);
  ledcWrite(9, 0);
  ledcWrite(4, 0);
  ledcWrite(5, speed);

  Serial.println("Rotating Left");
}

void MotorController::rotateRight(int speed) {
  speed = constrain(speed, 0, 1023);

  // Rotate Right
  ledcWrite(6, speed);
  ledcWrite(7, 0);
  ledcWrite(2, 0);
  ledcWrite(3, speed);
  ledcWrite(8, 0);
  ledcWrite(9, speed);
  ledcWrite(4, speed);
  ledcWrite(5, 0);

  Serial.println("Rotating Right");
}

void MotorController::moveDiagonalForwardLeft(int speed) {
  // Limit speed to half range so that 2*speed <= 1023
  speed = constrain(speed, 0, 511);

  // Forward-Left Diagonal Movement
  ledcWrite(6, speed); // M1_RPWM
  ledcWrite(7, 0);
  ledcWrite(2, speed); // M2_RPWM
  ledcWrite(3, 0);
  ledcWrite(8, 0);
  ledcWrite(9, 0);
  ledcWrite(4, 0);
  ledcWrite(5, 0);

  Serial.println("Diagonal Forward-Left");
}

void MotorController::moveDiagonalForwardRight(int speed) {
  speed = constrain(speed, 0, 511);

  // Forward-Right Diagonal Movement
  ledcWrite(6, 0);
  ledcWrite(7, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
  ledcWrite(8, speed); // M3_RPWM
  ledcWrite(9, 0);
  ledcWrite(4, speed); // M4_RPWM
  ledcWrite(5, 0);

  Serial.println("Diagonal Forward-Right");
}

void MotorController::moveDiagonalBackwardLeft(int speed) {
  speed = constrain(speed, 0, 511);

  // Backward-Left Diagonal Movement
  ledcWrite(6, 0);
  ledcWrite(7, speed); // M1_LPWM
  ledcWrite(2, speed); // M2_LPWM
  ledcWrite(3, 0);
  ledcWrite(8, 0);
  ledcWrite(9, 0);
  ledcWrite(4, 0);
  ledcWrite(5, 0);

  Serial.println("Diagonal Backward-Left");
}

void MotorController::moveDiagonalBackwardRight(int speed) {
  speed = constrain(speed, 0, 511);

  // Backward-Right Diagonal Movement
  ledcWrite(6, 0);
  ledcWrite(7, speed); // M1_LPWM
  ledcWrite(2, 0);
  ledcWrite(3, speed); // M2_LPWM
  ledcWrite(8, 0);
  ledcWrite(9, speed); // M3_LPWM
  ledcWrite(4, 0);
  ledcWrite(5, speed); // M4_LPWM

  Serial.println("Diagonal Backward-Right");
}

void MotorController::moveCircular(int translationalSpeed, int rotationalSpeed) {
  // Each motor = (translational ± rotational)
  translationalSpeed = constrain(translationalSpeed, -1023, 1023);
  rotationalSpeed    = constrain(rotationalSpeed,    -1023, 1023);

  // Calculate raw speeds
  int motor1 = translationalSpeed + rotationalSpeed; // M1 (Rear Left)
  int motor2 = translationalSpeed - rotationalSpeed; // M2 (Front Right)
  int motor3 = translationalSpeed - rotationalSpeed; // M3 (Rear Right)
  int motor4 = translationalSpeed + rotationalSpeed; // M4 (Front Left)

  // Constrain
  motor1 = constrain(motor1, -1023, 1023);
  motor2 = constrain(motor2, -1023, 1023);
  motor3 = constrain(motor3, -1023, 1023);
  motor4 = constrain(motor4, -1023, 1023);

  // Rear Left (M1)
  if (motor1 >= 0) {
    ledcWrite(6, motor1);
    ledcWrite(7, 0);
  } else {
    ledcWrite(6, 0);
    ledcWrite(7, abs(motor1));
  }

  // Front Right (M2)
  if (motor2 >= 0) {
    ledcWrite(2, motor2);
    ledcWrite(3, 0);
  } else {
    ledcWrite(2, 0);
    ledcWrite(3, abs(motor2));
  }

  // Rear Right (M3)
  if (motor3 >= 0) {
    ledcWrite(8, motor3);
    ledcWrite(9, 0);
  } else {
    ledcWrite(8, 0);
    ledcWrite(9, abs(motor3));
  }

  // Front Left (M4)
  if (motor4 >= 0) {
    ledcWrite(4, motor4);
    ledcWrite(5, 0);
  } else {
    ledcWrite(4, 0);
    ledcWrite(5, abs(motor4));
  }

  Serial.println("Moving in (Arc) Circular Trajectory");
}

void MotorController::moveCircularWithRadius(int linearPWM, float radius) {
  // linearPWM within ±1023
  linearPWM = constrain(linearPWM, -1023, 1023);

  // Example scale factor for computing rotation from radius
  float radiusScale = 100.0; 
  float rotational  = (float)linearPWM * (radiusScale / radius);
  int   rotPWM      = constrain((int)rotational, -1023, 1023);

  // Move using combined circular control
  moveCircular(linearPWM, rotPWM);

  Serial.print("Circular w/ Radius => linear:");
  Serial.print(linearPWM);
  Serial.print(", radius:");
  Serial.println(radius);
}

void MotorController::moveCircularCenterFacing(int linearPWM, float radius) {
  // Constrain linear
  linearPWM = constrain(linearPWM, -1023, 1023);

  // w = v / R  (plus optional scale factor)
  float scaleFactor = 10.0;
  float rawOmega    = float(linearPWM) / radius;
  float scaledOmega = rawOmega * scaleFactor;
  int rotationalPWM = constrain((int)scaledOmega, -1023, 1023);

  // Move
  moveCircular(linearPWM, rotationalPWM);

  Serial.print("Center-Facing Circle => v:");
  Serial.print(linearPWM);
  Serial.print(", R:");
  Serial.print(radius);
  Serial.print(", wPWM:");
  Serial.println(rotationalPWM);
}

void MotorController::moveSquare(int durationSeconds) {
  if (durationSeconds <= 0) {
    Serial.println("Invalid duration for Square Trajectory.");
    return;
  }

  Serial.println("Starting Square Trajectory...");

  // Move Forward
  moveForward(500); // Adjust speed as needed
  delay(durationSeconds * 1000);
  stopRobot();
  delay(500); // Brief pause between movements

  // Strafe Right
  strafeRight(500);
  delay(durationSeconds * 1000);
  stopRobot();
  delay(500);

  // Move Backward
  moveBackward(500);
  delay(durationSeconds * 1000);
  stopRobot();
  delay(500);

  // Strafe Left
  strafeLeft(500);
  delay(durationSeconds * 1000);
  stopRobot();
  delay(500);

  Serial.println("Completed Square Trajectory");
}

void MotorController::moveRobot(String direction) {
  direction.replace("\"", ""); // Clean up any quotes
  Serial.println("Command received: " + direction);

  if      (direction == "forward")              moveForward(500);
  else if (direction == "backward")             moveBackward(500);
  else if (direction == "left")                 strafeLeft(500);
  else if (direction == "right")                strafeRight(500);
  else if (direction == "rotateLeft")           rotateLeft(500);
  else if (direction == "rotateRight")          rotateRight(500);
  else if (direction == "stop")                 stopRobot();
  else if (direction == "circular")             moveCircular(500, 200);  // example
  else if (direction == "diagonalForwardLeft")  moveDiagonalForwardLeft(300);
  else if (direction == "diagonalForwardRight") moveDiagonalForwardRight(300);
  else if (direction == "diagonalBackwardLeft") moveDiagonalBackwardLeft(300);
  else if (direction == "diagonalBackwardRight")moveDiagonalBackwardRight(300);
  else {
    Serial.println("Unknown direction command.");
  }
}

void MotorController::setPWM(String motor, int pwmVal) {
  pwmVal = constrain(pwmVal, 0, 1023);
  bool found = false;
  for(int i = 0; i < NUM_MOTORS; i++) {
    if(motorPWMs[i].name == motor) {
      ledcWrite(motorPWMs[i].channel, pwmVal);
      motorPWMs[i].value = pwmVal;
      found = true;
      Serial.println("Set " + motor + " PWM to " + String(pwmVal));
      break;
    }
  }
  if(!found) {
    Serial.println("Unknown motor: " + motor);
  }
}

int MotorController::getPWM(String motor) {
  for(int i = 0; i < NUM_MOTORS; i++) {
    if(motorPWMs[i].name == motor) {
      return motorPWMs[i].value;
    }
  }
  return -1; // Indicates not found
}

void MotorController::getAllPWM(uint16_t* pwmValues) {
  for(int i = 0; i < NUM_MOTORS; i++) {
    pwmValues[i] = motorPWMs[i].value;
  }
}

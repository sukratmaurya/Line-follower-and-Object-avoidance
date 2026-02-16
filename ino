// INTEGRATED OBJECT AVOIDANCE + LINE FOLLOWING CAR
#include <NewPing.h>
#include <Servo.h>
#include <AFMotor.h>

// Define ultrasonic sensor pins
#define TRIGGER_PIN A5
#define ECHO_PIN A4
#define max_distance 50

// Define IR sensor pins
#define leftSensor   A0
#define centerSensor A1
#define rightSensor  A2

// Define speed profiles
#define LINE_FOLLOW_SPEED 100
#define OBSTACLE_AVOID_SPEED 130
#define MAX_SPEED 255
#define SEARCH_SPEED 80

// PID constants
float Kp = 30;
float Ki = 0.0;
float Kd = 20;
int lastError = 0;
float integral = 0;

Servo servo;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);

// Define motors
AF_DCMotor motor1(1, MOTOR12_1KHZ); 
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

// Global variables
int distance = 0;
int leftDistance;
int rightDistance;
boolean object;
boolean wasAvoiding = false;
boolean turnedLeft = false;
boolean obstacleDetected = false;
unsigned long lineLostTime = 0;
boolean searchingForLine = false;

void setup() {
  Serial.begin(9600);
  
  // Setup IR sensors
  pinMode(leftSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightSensor, INPUT);
  
  // Setup servo
  servo.attach(10);
  servo.write(90);
  
  // Initial speed setting
  setMotorSpeed(LINE_FOLLOW_SPEED);
}

void loop() {
  // Check for obstacles first
  checkObstacle();
  
  if (obstacleDetected) {
    // Object avoidance mode
    objectAvoid();
  } else {
    // Line following mode
    if (wasAvoiding) {
      // Just finished obstacle avoidance - help it find the line
      findLineAfterObstacle();
      wasAvoiding = false;
    }
    lineFollow();
  }
}

void checkObstacle() {
  distance = getDistance();
  if (distance <= 15) {
    obstacleDetected = true;
    setMotorSpeed(OBSTACLE_AVOID_SPEED);
  }
}

void objectAvoid() {
  distance = getDistance();
  
  if (distance <= 15) {
    Stop();
    wasAvoiding = true;

    lookLeft();
    lookRight();
    delay(100);
    
    if (rightDistance <= leftDistance) {
      object = true;
      turnedLeft = true;
      turn();
    } else {
      object = false;
      turnedLeft = false;
      turn();
    }
    delay(100);
  }
  else {
    // No more obstacles detected
    obstacleDetected = false;
    setMotorSpeed(LINE_FOLLOW_SPEED);
  }
}

void findLineAfterObstacle() {
  Serial.println("Finding line after obstacle...");
  searchingForLine = true;
  lineLostTime = millis();
  
  // First, try to go straight for a short time
  moveForward();
  delay(500);
  
  int attempts = 0;
  int maxAttempts = 10; // Increased attempts for better line finding
  
  while (attempts < maxAttempts && !isOnLine()) {
    Serial.println("Searching for line...");
    
    // Alternate between left and right sweeping motions
    if (attempts % 2 == 0) {
      // Turn slightly left
      moveLeft();
      delay(200);
      moveForward();
      delay(300);
    } else {
      // Turn slightly right
      moveRight();
      delay(200);
      moveForward();
      delay(300);
    }
    
    // Check if we found the line
    if (isOnLine()) {
      Serial.println("Line found!");
      break;
    }
    
    attempts++;
    delay(100);
  }
  
  // If still not found, do a wider search
  if (!isOnLine()) {
    Serial.println("Wider search pattern...");
    // Wider turn in the direction we originally turned for obstacle
    if (turnedLeft) {
      moveRight();
      delay(600);
    } else {
      moveLeft();
      delay(600);
    }
    moveForward();
    delay(800);
  }
  
  // Final check and stop
  Stop();
  delay(200);
  searchingForLine = false;
  Serial.println("Line search complete");
}

bool isOnLine() {
  int left = digitalRead(leftSensor);
  int center = digitalRead(centerSensor);
  int right = digitalRead(rightSensor);
  
  // Return true if any sensor detects the line
  // You can adjust this condition based on your line color
  // For black line on white surface: 0 = line detected
  // For white line on black surface: 1 = line detected
  return (left == 0 || center == 0 || right == 0);
}

void lineFollow() {
  int left = digitalRead(leftSensor);
  int center = digitalRead(centerSensor);
  int right = digitalRead(rightSensor);

  // If completely lost line for too long, trigger line search
  if (left == 1 && center == 1 && right == 1) {
    if (!searchingForLine) {
      if (lineLostTime == 0) {
        lineLostTime = millis();
      } else if (millis() - lineLostTime > 2000) { // 2 seconds without line
        findLineAfterObstacle();
        lineLostTime = 0;
        return;
      }
    }
  } else {
    lineLostTime = 0; // Reset if we're on line
  }

  // Convert sensor readings into position value
  int position = 0;
  if (left == 0 && center == 1 && right == 1) position = -2;
  else if (left == 0 && center == 0 && right == 1) position = -1; 
  else if (left == 1 && center == 0 && right == 1) position = 0;
  else if (left == 1 && center == 0 && right == 0) position = 1;
  else if (left == 1 && center == 1 && right == 0) position = 2;
  else if (left == 0 && center == 0 && right == 0) position = lastError > 0 ? 3 : -3;

  // Calculate error
  int error = position;

  // PID calculations
  integral += error;
  float derivative = error - lastError;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  // Calculate motor speeds
  int leftSpeed = LINE_FOLLOW_SPEED - correction;
  int rightSpeed = LINE_FOLLOW_SPEED + correction;

  // Limit speeds to safe range
  leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

  // Apply speeds for line following
  moveMotorsPID(leftSpeed, rightSpeed);

  // Debug info
  Serial.print("Mode: LINE | L:"); Serial.print(left);
  Serial.print(" C:"); Serial.print(center);
  Serial.print(" R:"); Serial.print(right);
  Serial.print(" | Error: "); Serial.print(error);
  Serial.print(" | Correction: "); Serial.println(correction);

  delay(10);
}

// ===== HELPER FUNCTIONS =====

void setMotorSpeed(int speed) {
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
}

void moveMotorsPID(int leftSpeed, int rightSpeed) {
  // Left side motors
  motor1.run(FORWARD);
  motor1.setSpeed(leftSpeed);
  motor2.run(FORWARD);
  motor2.setSpeed(leftSpeed);

  // Right side motors
  motor3.run(FORWARD);
  motor3.setSpeed(rightSpeed);
  motor4.run(FORWARD);
  motor4.setSpeed(rightSpeed);
}

int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 100;
  }
  return cm;
}

int lookLeft() {
  servo.write(150);
  delay(500);
  leftDistance = getDistance();
  delay(100);
  servo.write(90);
  Serial.print("Left:");
  Serial.print(leftDistance);
  return leftDistance;
}

int lookRight() {
  servo.write(30);
  delay(500);
  rightDistance = getDistance();
  delay(100);
  servo.write(90);
  return rightDistance;
}

void Stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void moveBackward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void turn() {
  if (object == false) {
    moveLeft();
    delay(700);
    moveForward();
    delay(800);
    moveRight();
    delay(900);
    moveForward();
  }
  else {
    moveRight();
    delay(700);
    moveForward();
    delay(800);
    moveLeft();
    delay(900);
    moveForward();
  }
}

void moveRight() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void moveLeft() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

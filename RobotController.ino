
#include <Servo.h> // Use standard Servo library for Arduino Nano

// === DC Motor Pins 
const int enableRightMotor = 5; // PWM pin (Timer2)
const int enableLeftMotor = 6;  // PWM pin (Timer0)

const int rightMotorPin1 = 2;
const int rightMotorPin2 = 3;
const int leftMotorPin1 = 4;
const int leftMotorPin2 = 7;

// === Motor speed ===
#define MAX_MOTOR_SPEED 125 // roughly half speed as example

// IR Sensor pins (analog pins used as digital)
const int irLeft = A0;
const int irRight = A1;

// === Gripper Servo ===
Servo gripper;
const int gripperPin = 9;
#define GRIPPER_OPEN_POS 0
#define GRIPPER_NEUTRAL_POS 90
#define GRIPPER_CLOSE_POS 45

// === Ultrasonic Sensor Pins ===
#define trigPin 10
#define echoPin 11

long duration;
int distance;

// === line tracking settings ===
bool lineTrackingEnabled = true;
bool isMoving = false;

// === Motor Functions ===

void turnLeft() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  analogWrite(enableRightMotor, MAX_MOTOR_SPEED);
  analogWrite(enableLeftMotor, MAX_MOTOR_SPEED);
}

void turnRight() {
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(enableRightMotor, MAX_MOTOR_SPEED);
  analogWrite(enableLeftMotor, MAX_MOTOR_SPEED);
}

void moveBackward() {
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  analogWrite(enableRightMotor, MAX_MOTOR_SPEED);
  analogWrite(enableLeftMotor, MAX_MOTOR_SPEED);
}

void moveForward() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(enableRightMotor, MAX_MOTOR_SPEED);
  analogWrite(enableLeftMotor, MAX_MOTOR_SPEED);
  if (!isMoving) {
    Serial.println("Moving forward");
    isMoving = true;
  }
}

void stopMotors() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  analogWrite(enableRightMotor, 0);
  analogWrite(enableLeftMotor, 0);
  if (isMoving) {
    Serial.println("Stopped");
    isMoving = false;
  }
}

// === Gripper functions ===
void gripperOpen() {
  gripper.write(GRIPPER_OPEN_POS);
}

void gripperClose() {
  gripper.write(GRIPPER_CLOSE_POS);
}

void gripperNeutral() {
  gripper.write(GRIPPER_NEUTRAL_POS);
}

// === Ultrasonic distance measurement function ===
void ultra() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

// === Menu print function ===
void printMenu() {
  Serial.println(F("\n=== Robot Status Menu ==="));
  Serial.print(F("Line Tracking: "));
  Serial.println(lineTrackingEnabled ? F("ENABLED") : F("DISABLED"));

  Serial.print(F("Turning: "));
  Serial.println(F("NO"));

  Serial.println(F("\nCommands:"));
  Serial.println(F("F - Move Forward"));
  Serial.println(F("B - Move Backward"));
  Serial.println(F("L - Turn Left"));
  Serial.println(F("R - Turn Right"));
  Serial.println(F("S - Stop"));
  Serial.println(F("O - Gripper Open"));
  Serial.println(F("C - Gripper Close"));
  Serial.println(F("N - Gripper Neutral"));
  Serial.println(F("T - Toggle Line Tracking"));
  Serial.println(F("========================\n"));
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Motor control pins
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(enableRightMotor, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);

  // IR sensor pins
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);

  // Start motors off
  stopMotors();

  // Setup servo
  gripper.attach(gripperPin);
  gripperOpen(); // start with gripper open
  printMenu();
}

void loop() {
  // Handle serial commands
  while (Serial.available()) {
    char cmd = toupper(Serial.read());
    if (cmd == '\n' || cmd == '\r' || cmd == ' ') continue;

    bool known = true;
    switch (cmd) {
      case 'F':
        lineTrackingEnabled = false;
        moveForward();
        break;
      case 'B':
        lineTrackingEnabled = false;
        moveBackward();
        break;
      case 'L':
        lineTrackingEnabled = false;
        turnLeft();
        break;
      case 'R':
        lineTrackingEnabled = false;
        turnRight();
        break;
      case 'S':
        lineTrackingEnabled = false;
        stopMotors();
        break;
      case 'O':
        lineTrackingEnabled = false;
        gripperOpen();
        Serial.println("Gripper Open");
        break;
      case 'C':
        lineTrackingEnabled = false;
        gripperClose();
        Serial.println("Gripper Closed");
        break;
      case 'N':
        lineTrackingEnabled = false;
        gripperNeutral();
        Serial.println("Gripper Neutral (90°)");
        break;
      case 'T':
        lineTrackingEnabled = !lineTrackingEnabled;
        if (lineTrackingEnabled) {
          Serial.println("Line tracking enabled");
        } else {
          stopMotors();
          Serial.println("Line tracking disabled");
        }
        break;
      case 'M':
        printMenu();
        break;
      default:
        known = false;
        break;
    }

    if (!known) {
      Serial.print("Unknown command: ");
      Serial.println(cmd);
      Serial.println("Use F,B,L,R,S,O,C,N,T");
    }
  }

  // === Line tracking logic ===
  if (lineTrackingEnabled) {
    bool leftDetected = digitalRead(irLeft) == LOW;   // LOW = line detected
    bool rightDetected = digitalRead(irRight) == LOW;

    if (leftDetected && rightDetected) {
      moveForward();
    } else if (leftDetected) {
      turnLeft();
    } else if (rightDetected) {
      turnRight();
    } else {
      stopMotors();
    }
  }

  // === IR Sensor Status Printing ===
  static unsigned long lastPrintTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime > 300) {  // print every 300ms
    int irLeftValue = digitalRead(irLeft);
    int irRightValue = digitalRead(irRight);

    Serial.print("IR Left: ");
    Serial.print(irLeftValue == LOW ? "LINE " : "NO LINE ");
    Serial.print(" | IR Right: ");
    Serial.println(irRightValue == LOW ? "LINE" : "NO LINE");

    lastPrintTime = currentTime;
  }
}

#include <ESP32Servo.h>

// === Motor Pins ===
int enableRightMotor = 22;
int rightMotorPin1 = 18;
int rightMotorPin2 = 19;

int enableLeftMotor = 23;
int leftMotorPin1 = 17;
int leftMotorPin2 = 5;

// IR Sensor pins
const int irLeft = 34;
const int irRight = 35;

// === Gripper Servo ===
Servo gripper;
int gripperPin = 21;
#define GRIPPER_OPEN_POS 0
#define GRIPPER_NEUTRAL_POS 90
#define GRIPPER_CLOSE_POS 180

// === PWM Settings ===
const int PWMFreq = 1000;
const int PWMResolution = 8;
const int PWMRightChannel = 4;
const int PWMLeftChannel = 5;
#define MAX_MOTOR_SPEED 125

// === Grid Navigation Variables ===
int currentX = 0;      // Current X position (0-3)
int currentY = 0;      // Current Y position (0-3)
int targetX = 1;       // Target X position
int targetY = 2;       // Target Y position
int currentDirection = 0; // 0=North(+Y), 1=East(+X), 2=South(-Y), 3=West(-X)

// Navigation states
enum RoverState {
  MANUAL_CONTROL,
  GRID_NAVIGATION,
  FOLLOWING_LINE,
  AT_INTERSECTION,
  TURNING,
  REACHED_TARGET,
  RETURNING_HOME
};

RoverState state = MANUAL_CONTROL;
bool gridMissionActive = false;
bool returningHome = false;

// Timing variables for intersection detection
unsigned long lastMoveTime = 0;
unsigned long intersectionDelay = 2500; // Time between intersections (adjust based on speed)

void setup() {
  Serial.begin(115200);
  Serial.println("Enhanced Robot ready!");
  Serial.println("Commands: F, B, L, R, S, O, C, N, G (start grid mission)");

  // Setup motor pins
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(enableRightMotor, OUTPUT);

  // Setup IR sensor pins
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);

  // Setup PWM for motors
  ledcSetup(PWMRightChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableRightMotor, PWMRightChannel);
  ledcWrite(PWMRightChannel, MAX_MOTOR_SPEED);

  ledcSetup(PWMLeftChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableLeftMotor, PWMLeftChannel);
  ledcWrite(PWMLeftChannel, MAX_MOTOR_SPEED);

  // Attach gripper
  gripper.attach(gripperPin);
  gripperOpen(); // Start open

  stopMotors();
  
  Serial.println("Place robot at position (0,0) facing North");
  Serial.println("Press 'G' to start grid navigation mission: (0,0) -> (1,2) -> (0,0)");
}

void loop() {
  // Check for serial commands
  if (Serial.available()) {
    char cmd = toupper(Serial.read());
    handleSerialCommand(cmd);
  }

  // Execute current state
  switch(state) {
    case MANUAL_CONTROL:
      // Do nothing, wait for commands
      break;
      
    case GRID_NAVIGATION:
      executeGridNavigation();
      break;
      
    case FOLLOWING_LINE:
      followLine();
      break;
      
    case AT_INTERSECTION:
      handleIntersection();
      break;
      
    case TURNING:
      // Turning is handled in handleIntersection
      break;
      
    case REACHED_TARGET:
      handleTargetReached();
      break;
      
    case RETURNING_HOME:
      executeReturnHome();
      break;
  }
}

void handleSerialCommand(char cmd) {
  switch (cmd) {
    case 'F':
      state = MANUAL_CONTROL;
      moveForward();
      Serial.println("Manual: Forward");
      break;
    case 'B':
      state = MANUAL_CONTROL;
      moveBackward();
      Serial.println("Manual: Backward");
      break;
    case 'L':
      state = MANUAL_CONTROL;
      turnLeft();
      Serial.println("Manual: Left");
      break;
    case 'R':
      state = MANUAL_CONTROL;
      turnRight();
      Serial.println("Manual: Right");
      break;
    case 'S':
      state = MANUAL_CONTROL;
      stopMotors();
      Serial.println("Manual: Stop");
      break;
    case 'O':
      gripperOpen();
      Serial.println("Gripper Open");
      break;
    case 'C':
      gripperClose();
      Serial.println("Gripper Closed");
      break;
    case 'N':
      gripperNeutral();
      Serial.println("Gripper Neutral (90°)");
      break;
    case 'G': // Start grid navigation mission
      startGridMission();
      break;
    default:
      Serial.println("Commands: F, B, L, R, S, O, C, N, G (grid mission)");
      break;
  }
}

void startGridMission() {
  Serial.println("=== STARTING GRID MISSION ===");
  Serial.println("Mission: (0,0) -> (1,2) -> (0,0)");
  
  // Reset position and state
  currentX = 0;
  currentY = 0;
  currentDirection = 0; // Facing North
  targetX = 1;
  targetY = 2;
  returningHome = false;
  gridMissionActive = true;
  
  state = GRID_NAVIGATION;
  lastMoveTime = millis();
  
  Serial.println("Starting navigation to (1,2)...");
}

void executeGridNavigation() {
  if (!gridMissionActive) return;
  
  // Check if we're already at target
  if (currentX == targetX && currentY == targetY) {
    state = REACHED_TARGET;
    return;
  }
  
  // Start following the line
  state = FOLLOWING_LINE;
}

void followLine() {
  int leftSensor = digitalRead(irLeft);
  int rightSensor = digitalRead(irRight);
  
  // Check for intersection detection
  if (checkForIntersection()) {
    stopMotors();
    delay(300); // Brief pause at intersection
    state = AT_INTERSECTION;
    return;
  }
  
  // Basic line following logic
  if (leftSensor == LOW && rightSensor == LOW) {
    // Both sensors on line - go straight
    moveForward();
  }
  else if (leftSensor == LOW && rightSensor == HIGH) {
    // Left sensor on line - turn slightly left
    turnSlightLeft();
  }
  else if (leftSensor == HIGH && rightSensor == LOW) {
    // Right sensor on line - turn slightly right
    turnSlightRight();
  }
  else {
    // Both sensors off line - try to find it
    moveForward(); // Continue forward briefly
  }
}

bool checkForIntersection() {
  // Simple time-based intersection detection
  // In a real implementation, you'd use additional sensors
  
  unsigned long currentTime = millis();
  if (currentTime - lastMoveTime >= intersectionDelay) {
    lastMoveTime = currentTime;
    return true;
  }
  return false;
}

void handleIntersection() {
  Serial.print("At intersection (");
  Serial.print(currentX);
  Serial.print(",");
  Serial.print(currentY);
  Serial.println(")");
  
  // Calculate which direction to turn
  int nextDirection = calculateNextDirection();
  
  // Turn to face the correct direction
  turnToDirection(nextDirection);
  
  // Update position after turning
  updatePosition();
  
  // Continue following line
  state = FOLLOWING_LINE;
}

int calculateNextDirection() {
  int deltaX = targetX - currentX;
  int deltaY = targetY - currentY;
  
  // Simple pathfinding: prioritize X movement, then Y
  if (deltaX > 0) {
    return 1; // East (+X)
  } else if (deltaX < 0) {
    return 3; // West (-X)
  } else if (deltaY > 0) {
    return 0; // North (+Y)
  } else if (deltaY < 0) {
    return 2; // South (-Y)
  }
  
  return currentDirection; // No movement needed
}

void turnToDirection(int targetDirection) {
  int turnAmount = (targetDirection - currentDirection + 4) % 4;
  
  Serial.print("Current direction: ");
  Serial.print(getDirectionName(currentDirection));
  Serial.print(" -> Target direction: ");
  Serial.println(getDirectionName(targetDirection));
  
  switch(turnAmount) {
    case 0: // No turn needed
      Serial.println("No turn needed");
      break;
      
    case 1: // Turn right 90°
      Serial.println("Turning right 90°");
      performRightTurn();
      break;
      
    case 2: // Turn 180°
      Serial.println("Turning 180°");
      performRightTurn();
      delay(100);
      performRightTurn();
      break;
      
    case 3: // Turn left 90°
      Serial.println("Turning left 90°");
      performLeftTurn();
      break;
  }
  
  currentDirection = targetDirection;
}

void updatePosition() {
  // Update position based on the direction we're moving
  switch(currentDirection) {
    case 0: // North
      currentY++;
      break;
    case 1: // East
      currentX++;
      break;
    case 2: // South
      currentY--;
      break;
    case 3: // West
      currentX--;
      break;
  }
  
  Serial.print("Moving to position (");
  Serial.print(currentX);
  Serial.print(",");
  Serial.print(currentY);
  Serial.println(")");
}

void handleTargetReached() {
  Serial.println("=== TARGET REACHED ===");
  stopMotors();
  
  if (!returningHome) {
    Serial.println("Reached (1,2)! Now returning home to (0,0)...");
    
    // Set new target as home
    targetX = 0;
    targetY = 0;
    returningHome = true;
    
    delay(2000); // Pause at target
    state = RETURNING_HOME;
  } else {
    Serial.println("=== MISSION COMPLETE ===");
    Serial.println("Successfully returned to (0,0)");
    gridMissionActive = false;
    state = MANUAL_CONTROL;
    
    // Celebrate with gripper movement
    gripperClose();
    delay(500);
    gripperOpen();
    delay(500);
    gripperNeutral();
  }
}

void executeReturnHome() {
  executeGridNavigation(); // Same logic as going to target
}

String getDirectionName(int dir) {
  switch(dir) {
    case 0: return "North";
    case 1: return "East";
    case 2: return "South";
    case 3: return "West";
    default: return "Unknown";
  }
}

// === Enhanced Motor Movement Functions ===
void moveBackward() {
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
}

void moveForward() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
}

void turnLeft() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);
}

void turnRight() {
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
}

void turnSlightLeft() {
  // Slow down right motor for gentle left turn
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  
  // Reduce right motor speed
  ledcWrite(PWMRightChannel, MAX_MOTOR_SPEED * 0.6);
  ledcWrite(PWMLeftChannel, MAX_MOTOR_SPEED);
}

void turnSlightRight() {
  // Slow down left motor for gentle right turn
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);
  
  // Reduce left motor speed
  ledcWrite(PWMRightChannel, MAX_MOTOR_SPEED);
  ledcWrite(PWMLeftChannel, MAX_MOTOR_SPEED * 0.6);
}

void performRightTurn() {
  turnRight();
  delay(800); // Adjust timing for 90° turn
  stopMotors();
  delay(200);
  
  // Reset motor speeds
  ledcWrite(PWMRightChannel, MAX_MOTOR_SPEED);
  ledcWrite(PWMLeftChannel, MAX_MOTOR_SPEED);
}

void performLeftTurn() {
  turnLeft();
  delay(800); // Adjust timing for 90° turn
  stopMotors();
  delay(200);
  
  // Reset motor speeds
  ledcWrite(PWMRightChannel, MAX_MOTOR_SPEED);
  ledcWrite(PWMLeftChannel, MAX_MOTOR_SPEED);
}

void stopMotors() {
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);
  
  // Reset motor speeds to full
  ledcWrite(PWMRightChannel, MAX_MOTOR_SPEED);
  ledcWrite(PWMLeftChannel, MAX_MOTOR_SPEED);
}

// === Gripper Functions ===
void gripperOpen() {
  gripper.write(GRIPPER_OPEN_POS);
}

void gripperClose() {
  gripper.write(GRIPPER_CLOSE_POS);
}

void gripperNeutral() {
  gripper.write(GRIPPER_NEUTRAL_POS);
}

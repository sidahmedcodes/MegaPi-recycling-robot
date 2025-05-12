/*
 * Hey there! This is our Arduino code for the waste sorting robot.
 * It handles all the robot movement, arm control, Bluetooth stuff,
 * and talks to the Raspberry Pi using UART. Basically the brains of 
 * the operation for motor control and physical actions.
 * 
 * We've kept the architecture pretty simple by unifying our approach
 * for both objects and bins - makes the code way cleaner!
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AFMotor.h>

// Initialize motors using Adafruit Motor Shield
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// Ultrasonic sensor pins
#define TRIG_PIN 22
#define ECHO_PIN 23

// Servo channel setup - these are the PWM channels on the driver board
#define SERVO_FREQ 50 // Standard 50Hz for RC servos
#define BASE_CHANNEL 0
#define SHOULDER_CHANNEL 4
#define ELBOW_CHANNEL 8
#define WRIST_ROT_CHANNEL 10
#define WRIST_UD_CHANNEL 15
#define GRIPPER_CHANNEL 14

// Arm positions for different actions - these took FOREVER to calibrate!
#define SCAN_BASE 55
#define SCAN_SHOULDER 60
#define SCAN_ELBOW 100
#define SCAN_WRIST_ROT 80
#define SCAN_WRIST_UD 25
#define SCAN_GRIPPER 120

#define PICKUP_BASE 55
#define PICKUP_SHOULDER 23
#define PICKUP_ELBOW 90
#define PICKUP_WRIST_ROT 80
#define PICKUP_WRIST_UD 25
#define PICKUP_GRIPPER 10

#define CARRY_BASE 55
#define CARRY_SHOULDER 60
#define CARRY_ELBOW 100
#define CARRY_WRIST_ROT 80
#define CARRY_WRIST_UD 10
#define CARRY_GRIPPER 10

#define DROPOFF_BASE 55
#define DROPOFF_SHOULDER 60
#define DROPOFF_ELBOW 75
#define DROPOFF_WRIST_ROT 80
#define DROPOFF_WRIST_UD 25
#define DROPOFF_GRIPPER 120

// Special position for going forward after finding the bin with camera
#define FORWARD_ELBOW 75

// Some important constants we figured out through testing
#define DEFAULT_SPEED 200
#define TURN_MS_PER_DEGREE 4.3  // Roughly 4.3ms per degree at speed 200
#define MIN_DISTANCE 7  // Too close! Emergency stop at 7cm
#define OBJECT_STOP_DISTANCE 18.0 // Sweet spot for grabbing objects
#define BIN_STOP_DISTANCE 11.0 // Sweet spot for dropping in bins

// Create servo driver object
Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

// --- State Machine Stuff ---
// Operating modes - either manual control or autonomous
enum Mode {
  MANUAL,
  AUTO
};

// What we're currently targeting - either an object to pick up or a bin to drop in
enum Target {
  OBJECT,
  BIN_TARGET
};

// Our robot's brain states - what it's currently doing
enum RobotState {
  IDLE,         // Just chilling, waiting for commands
  SEARCH,       // Looking around for something interesting
  ALIGN_ROUGH,  // Found something! Turning roughly towards it
  ALIGN_FINE,   // Fine-tuning our alignment (camera helps here)
  APPROACH,     // Moving forward until we're close enough
  ACTION,       // Doing the pickup or dropoff
  TRANSITION,   // Switching between targets
  COMPLETE      // All done, mission accomplished!
};

// Global variables
Mode currentMode = MANUAL;
Target currentTarget = OBJECT;
RobotState currentState = IDLE;
RobotState lastReportedState = IDLE;  // For debugging

boolean waitingForRPiResponse = false;
unsigned long lastCommandTime = 0;
unsigned long lastStatusMessageTime = 0; // For throttling app messages
int manualSpeed = 150;  // Default manual speed (medium)

// Initialize static variables for state tracking
boolean actionCompleted = false;  // Flag for tracking action completion

void setup() {
  // Initialize serial communications
  Serial.begin(115200);    // Debug serial
  Serial1.begin(9600);     // Bluetooth serial
  Serial3.begin(115200);   // UART to Raspberry Pi
  
  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Initialize servo driver
  servoDriver.begin();
  servoDriver.setOscillatorFrequency(25000000);
  servoDriver.setPWMFreq(SERVO_FREQ);
  
  // Set initial motor speeds
  motor1.setSpeed(manualSpeed);
  motor2.setSpeed(manualSpeed);
  motor3.setSpeed(manualSpeed);
  motor4.setSpeed(manualSpeed);
  
  // Set arm to initial position
  moveArmToScanPosition();
  
  // Stop all motors initially
  stopMotors();
  
  Serial.println("Arduino initialized. Ready for commands.");
  Serial1.println("System ready");
}

void loop() {
  // Check for Bluetooth commands from the app
  if (Serial1.available() > 0) {
    char command = Serial1.read();
    processBluetoothCommand(command);
  }
  
  // Check for commands from Raspberry Pi
  if (Serial3.available() > 0) {
    String command = Serial3.readStringUntil('\n');
    processRaspberryPiCommand(command);
  }
  
  // Check for Serial monitor debug commands
  if (Serial.available() > 0) {
    processSerialDebugCommand();
  }
  
  // Always send status messages to app in AUTO mode
  if (currentMode == AUTO) {
    sendStatusMessage();
    
    // Print debug distance info every second
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 1000) {
      float distance = getDistance();
      String targetName = (currentTarget == OBJECT) ? "OBJECT" : "BIN";
      String stateName = getStateName(currentState);
      Serial.println("DEBUG - Distance: " + String(distance) + " cm, Target: " + targetName + ", State: " + stateName);
      lastDebugTime = millis();
    }
  }
  
  // Execute auto mode state machine when not waiting for RPi
  if (currentMode == AUTO && !waitingForRPiResponse) {
    executeAutoMode();
  }
}

// Process commands from the bluetooth app
void processBluetoothCommand(char command) {
  if (command == 'A') {
    // Activate Auto Mode
    if (currentMode == MANUAL) {
      activateAutoMode();
    }
  } else if (command == 'O') {
    // Deactivate Auto Mode
    if (currentMode == AUTO) {
      deactivateAutoMode();
    }
  } else if (currentMode == MANUAL) {
    // Handle manual mode movement commands
    processManualMovementCommand(command);
  }
}

void activateAutoMode() {
  currentMode = AUTO;
  currentTarget = OBJECT;
  currentState = SEARCH;
  Serial.println("Activating Auto Mode - Target: OBJECT, State: SEARCH");
  
  // Clear any pending messages
  while (Serial1.available()) Serial1.read();
  
  // Signal app and RPi
  Serial1.println("AUTO_START?");
  delay(400);
  Serial3.println("AUTO_START");
  
  // Move arm to scan position
  moveArmToScanPosition();
}

void deactivateAutoMode() {
  currentMode = MANUAL;
  stopMotors();
  Serial.println("Deactivating Auto Mode");
  
  // Clear any pending messages
  while (Serial1.available()) Serial1.read();
  
  // Signal app and RPi
  Serial1.println("MANUAL_MODE?");
  delay(400);
  Serial3.println("AUTO_STOP");
}

void processManualMovementCommand(char command) {
  switch (command) {
    case 'G': // Forward
      moveForward(manualSpeed);
      break;
    case 'H': // Backward
      moveBackward(manualSpeed);
      break;
    case 'I': // Left
      turnLeft(manualSpeed);
      break;
    case 'J': // Right
      turnRight(manualSpeed);
      break;
    case 'K': // Stop
      stopMotors();
      break;
    // Speed control commands
    case 'Z': // High speed
      manualSpeed = 255;
      updateRunningMotorSpeeds(manualSpeed);
      break;
    case 'Y': // Medium speed
      manualSpeed = 150;
      updateRunningMotorSpeeds(manualSpeed);
      break;
    case 'X': // Low speed
      manualSpeed = 100;
      updateRunningMotorSpeeds(manualSpeed);
      break;
  }
}

void processSerialDebugCommand() {
  char input = Serial.read();
  if (input == 'N' || input == 'n') {
    // Send NEXT_STATE command to RPi
    Serial.println("Manual state advancement requested");
    Serial3.println("NEXT_STATE");
  } else if (input == 'D' || input == 'd') {
    // Force a distance measurement
    float distance = getDistance();
    Serial.println("Manual distance check: " + String(distance) + " cm");
  } else if (input == 'S' || input == 's') {
    // Emergency stop
    stopMotors();
    Serial.println("EMERGENCY STOP from serial monitor");
  } else if (input == 'O' || input == 'o') {
    // Force transition to OBJECT target
    currentTarget = OBJECT;
    currentState = SEARCH;
    Serial.println("Forced target change to OBJECT");
  } else if (input == 'B' || input == 'b') {
    // Force transition to BIN target
    currentTarget = BIN_TARGET;
    currentState = SEARCH;
    Serial.println("Forced target change to BIN");
  }
}

// Process commands from the Raspberry Pi
void processRaspberryPiCommand(String command) {
  Serial.println("Received from RPi: " + command);
  
  if (command.startsWith("TURN:")) {
    // Format: TURN:angle (positive = right, negative = left)
    int angle = command.substring(5).toInt();
    executeTurn(angle);
    Serial3.println("TURN_COMPLETE");
    // After completing the turn, if in ALIGN states, move directly to APPROACH
    if (currentState == ALIGN_ROUGH || currentState == ALIGN_FINE) {
      Serial3.println("DETECTION_LOCK"); // Tell RPi to stop aruco detection
      Serial.println("Alignment complete, locking detection");
      delay(300); // Increased delay to ensure lock command is processed
      Serial3.println("DETECTION_LOCK"); // Send twice to be extra sure
      // Add a short delay before starting approach
      delay(500);
      
      // Set elbow to forward position before moving forward
      moveServo(ELBOW_CHANNEL, FORWARD_ELBOW);
      
      // Auto-start approach after alignment is done
      currentState = APPROACH;
      moveForward(DEFAULT_SPEED);
      Serial3.println("APPROACH_STARTED");
      // Send detection lock one more time after approach started
      delay(200);
      Serial3.println("DETECTION_LOCK");
    }
    waitingForRPiResponse = false;
  } 
  else if (command == "APPROACH") {
    // Start approaching the current target
    Serial.println("Starting approach toward " + String(currentTarget == OBJECT ? "OBJECT" : "BIN"));
    
    // Set elbow to 60 degrees position before moving forward if approaching after ArUco alignment
    if (currentTarget == BIN_TARGET) {
      moveServo(ELBOW_CHANNEL, FORWARD_ELBOW);
      
      // Ensure we disable detection completely
      Serial3.println("DETECTION_LOCK");
      delay(200); // Small delay to ensure command is processed
      Serial3.println("DETECTION_LOCK"); // Send twice to be extra sure
    }
    
    currentState = APPROACH;
    moveForward(DEFAULT_SPEED);
    // Tell RPi we're in approach mode, don't send further aruco detections
    Serial3.println("APPROACH_STARTED");
    Serial3.println("DETECTION_LOCK");
    waitingForRPiResponse = false;
  }
  else if (command == "STOP") {
    stopMotors();
    Serial3.println("STOP_COMPLETE");
    waitingForRPiResponse = false;
  }
  else if (command == "PICKUP") {
    performPickup();
    waitingForRPiResponse = false;
  }
  else if (command == "DROPOFF") {
    // Immediately execute dropoff when commanded
    performDropoff();
    waitingForRPiResponse = false;
  }
  else if (command == "SWITCH_TO_BIN") {
    // Switch target from OBJECT to BIN
    currentTarget = BIN_TARGET;
    currentState = SEARCH;
    Serial.println("Switching target to BIN, state to SEARCH");
    Serial3.println("TARGET_SWITCHED_TO_BIN");
    waitingForRPiResponse = false;
  }
  else if (command == "BACKWARD") {
    moveBackward(DEFAULT_SPEED);
    delay(1500);
    stopMotors();
    Serial3.println("BACKWARD_COMPLETE");
    waitingForRPiResponse = false;
  }
  else if (command == "DETECTION_LOCK") {
    // Explicit acknowledgment of detection lock command
    Serial.println("Acknowledging detection lock");
    Serial3.println("DETECTION_LOCK_ACKNOWLEDGED");
    waitingForRPiResponse = false;
  }
  else if (command.startsWith("SPEED:")) {
    // Format: SPEED:value (0-255)
    int speedValue = command.substring(6).toInt();
    if (speedValue >= 0 && speedValue <= 255) {
      setMotorSpeeds(speedValue);
      Serial3.println("SPEED_SET");
    } else {
      Serial3.println("INVALID_SPEED");
    }
    waitingForRPiResponse = false;
  }
  else if (command == "DISTANCE") {
    float distance = getDistance();
    Serial3.println("Distance: " + String(distance) + " cm");
    waitingForRPiResponse = false;
  }
  else if (command == "SCAN") {
    moveArmToScanPosition();
    Serial3.println("SCAN_COMPLETE");
    waitingForRPiResponse = false;
  }
  else if (command == "CARRY") {
    moveArmToCarryPosition();
    Serial3.println("CARRY_COMPLETE");
    waitingForRPiResponse = false;
  }
  else if (command == "NEXT_STATE") {
    // For debugging
    waitingForRPiResponse = false;
  }
  else {
    // Unknown command
    Serial.println("Unknown command: " + command);
    waitingForRPiResponse = false;
  }
}

// Where the magic happens for the autonomous mode
void executeAutoMode() {
  // Keep track of the previous state so we can detect transitions
  RobotState previousState = lastReportedState;
  
  // Let everyone know when we're changing states
  reportStateChange();
  
  // Special case when we're moving from alignment to approach mode
  if ((previousState == ALIGN_ROUGH || previousState == ALIGN_FINE) && currentState == APPROACH) {
    if (currentTarget == BIN_TARGET) {
      // Need to adjust the arm position before driving forward
      moveServo(ELBOW_CHANNEL, FORWARD_ELBOW);
      Serial.println("ArUco alignment complete - set elbow to forward position (60°)");
    }
  }
  
  switch (currentState) {
    case SEARCH:
      // The Pi is looking for stuff, we just wait patiently
      waitingForRPiResponse = true;
      break;
      
    case ALIGN_ROUGH:
    case ALIGN_FINE:
      // Let the Pi do its computer vision magic and tell us where to turn
      waitingForRPiResponse = true;
      break;
      
    case APPROACH:
      // This is where we actually drive forward until we're close enough
      executeApproach();
      break;
      
    case ACTION:
      // We already did the action in the transition, just waiting now
      waitingForRPiResponse = true;
      break;
      
    case TRANSITION:
      // Time to switch between tasks
      if (currentTarget == OBJECT && !actionCompleted) {
        // We just grabbed something, now let's find a bin for it
        currentTarget = BIN_TARGET;
        currentState = SEARCH;
        actionCompleted = true;
        Serial.println("Transitioning to BIN search");
        
        // Tell the Pi we're ready to look for bins now
        Serial3.println("READY_FOR_BIN_SEARCH");
      }
      else {
        // We're all done!
        currentState = COMPLETE;
      }
      break;
      
    case COMPLETE:
      // Victory lap - we've completed the task
      if (!actionCompleted) {
        Serial.println("Task complete!");
        Serial3.println("TASK_COMPLETE");
        actionCompleted = true;
      }
      break;
      
    default:
      break;
  }
}

// Moving toward our target until we're close enough
void executeApproach() {
  float distance = getDistance();
  
  // Different targets need different stopping distances
  float stopDistance = (currentTarget == OBJECT) ? OBJECT_STOP_DISTANCE : BIN_STOP_DISTANCE;
  
  // Keep us updated on how things are going
  String targetType = (currentTarget == OBJECT) ? "OBJECT" : "BIN";
  Serial.println("Approach " + targetType + " - distance: " + String(distance) + 
                " cm, stop at: " + String(stopDistance) + " cm");
  
  // Let the Pi know our current distance
  Serial3.println("Distance: " + String(distance) + " cm");
  
  // Safety first! Emergency stop if we're WAY too close
  if (distance <= MIN_DISTANCE) {
    stopMotors();
    Serial.println("EMERGENCY STOP - Too close! Distance: " + String(distance) + " cm");
    
    // If we had to emergency stop at a bin, try to recover by dropping off anyway
    if (currentTarget == BIN_TARGET) {
      Serial.println("Forcing bin reached notification due to emergency stop");
      Serial3.println("BIN_DISTANCE_REACHED");
      performDropoff();
    }
    return;
  }
  
  if (distance <= stopDistance) {
    // Stop motors when target distance reached
    stopMotors();
    Serial.println("TARGET REACHED: " + String(distance) + " cm");
    
    // Print additional details for debugging
    String msg = (currentTarget == OBJECT) ? 
                "Object reached - executing pickup" : 
                "Bin reached - executing dropoff";
    Serial.println(msg);
    
    // Send appropriate notification to RPi
    if (currentTarget == OBJECT) {
      Serial3.println("TARGET_DISTANCE_REACHED");
      // Auto-execute pickup sequence
      performPickup();
    } else { // BIN_TARGET
      Serial3.println("BIN_DISTANCE_REACHED");
      // Auto-execute dropoff immediately
      performDropoff();
      // currentState is set to TRANSITION within performDropoff()
    }
  }
  else if (distance <= (stopDistance + 5.0)) {
    // When very close to the target, move very slowly with careful distance checks
    int slowSpeed = 50;  // Very slow approach speed
    setMotorSpeeds(slowSpeed);
    Serial.println("Final approach - very slow speed: " + String(slowSpeed));
  }
  else {
    // Adjust speed based on distance
    adjustSpeedBasedOnDistance(distance, stopDistance);
  }
}

// Fancy distance-based speed control to make smoother approaches
void adjustSpeedBasedOnDistance(float distance, float stopDistance) {
  int speed;
  
  // Bins need extra careful approach - don't want to crash into them!
  if (currentTarget == BIN_TARGET) {
    // Slow down more aggressively as we get closer to the bin
    if (distance < 20) {
      // Creeping forward soooo slowly
      speed = 60;
    }
    else if (distance < 30) {
      // Pretty close, take it easy
      speed = 80;
    }
    else if (distance <= 50) {
      // Getting there, moderate pace
      speed = 120;
    }
    else {
      // Still far away, let's make some progress
      speed = 150;
    }
  } else {
    // Objects are a bit more forgiving for approach
    if (distance < 35) {
      // Getting close, slow down
      speed = 85;
    }
    else if (distance <= 50) {
      // Medium distance, steady pace
      speed = 100;
    }
    else {
      // Still far away, let's go faster
      speed = 150;
    }
  }
  
  // Update all motors to the new speed
  setMotorSpeeds(speed);
  
  // Only log when the speed actually changes to avoid spamming
  static int lastSpeed = 0;
  if (speed != lastSpeed) {
    Serial.println("Adjusting speed to " + String(speed) + " at distance " + String(distance) + " cm");
    lastSpeed = speed;
  }
}

// Perform complete pickup sequence
void performPickup() {
  Serial.println("Executing pickup sequence");
  
  // Stop motors if still moving
  stopMotors();
  
  // Move arm to pickup position
  moveArmToPickupPosition();
  Serial.println("Pickup complete");
  Serial3.println("PICKUP_COMPLETE");
  
  // Wait a moment to stabilize
  delay(1000);
  
  // Move arm to carry position
  moveArmToCarryPosition();
  Serial.println("Object in carry position");
  Serial3.println("CARRY_COMPLETE");
  
  // Transition to next phase
  currentState = TRANSITION;
  actionCompleted = false;
}

// Perform dropoff sequence
void performDropoff() {
  Serial.println("Executing dropoff sequence immediately");
  
  // Stop motors if still moving
  stopMotors();
  
  // Move arm to dropoff position
  moveArmToDropoffPosition();
  Serial.println("Dropoff complete");
  Serial3.println("DROPOFF_COMPLETE");
  
  // Transition to complete
  currentState = TRANSITION;
  actionCompleted = false;
}

// Motor control functions
void moveForward(int speed) {
  setMotorSpeeds(speed);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void moveBackward(int speed) {
  setMotorSpeeds(speed);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void turnLeft(int speed) {
  setMotorSpeeds(speed);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void turnRight(int speed) {
  setMotorSpeeds(speed);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void setMotorSpeeds(int speed) {
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed);
}

void executeTurn(int angle) {
  // Positive angle = turn right, negative angle = turn left
  unsigned long turnTime = abs(angle) * TURN_MS_PER_DEGREE;
  
  Serial.println("Turning " + String(angle) + " degrees, time: " + String(turnTime) + "ms");
  
  if (angle > 0) {
    turnRight(DEFAULT_SPEED);
  } else {
    turnLeft(DEFAULT_SPEED);
  }
  
  delay(turnTime);
  stopMotors();
  Serial.println("Turn completed");
  delay(1000);  // Small pause after turn
}

// Update motor speeds without changing direction
void updateRunningMotorSpeeds(int speed) {
  setMotorSpeeds(speed);
}

// Our trusty distance sensor - returns cm
float getDistance() {
  // Taking multiple readings makes this way more reliable
  const int numReadings = 5;  // Bumped this up from 3 to 5 for better results
  float readings[numReadings];
  int validReadings = 0;
  float sum = 0;
  
  // Let's take a few measurements and average them
  for (int i = 0; i < numReadings; i++) {
    // Reset the trigger pin
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    
    // Send the ultrasonic pulse
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Wait for the echo and measure the time
    long duration = pulseIn(ECHO_PIN, HIGH, 20000); // 20ms timeout
    
    // Only use readings that didn't time out
    if (duration > 0) {
      // Convert time to distance (sound travels at ~343m/s)
      float distance = duration * 0.034 / 2;  
      
      // Filter out obviously wrong values
      if (distance <= 400.0 && distance >= 2.0) {  
        readings[validReadings] = distance;
        sum += distance;
        validReadings++;
      }
    }
    
    // A little breather between readings
    delay(20);  
  }
  
  // If we got some valid readings, average them out
  if (validReadings > 0) {
    float avgDistance = sum / validReadings;
    
    // A little calibration hack for approaching bins
    if (currentTarget == BIN_TARGET && currentState == APPROACH) {
      // We found we need to be a bit closer than the sensor thinks
      avgDistance -= 2.0;  
    }
    
    return avgDistance;
  } else {
    // Something's gone wrong with the sensor
    Serial.println("WARNING: No valid ultrasonic readings");
    return 1000.0;  // Return a large value to indicate "very far away"
  }
}

// ARM CONTROL FUNCTIONS
// Function to detach servo
void detachServo(int channel) {
  servoDriver.setPWM(channel, 0, 0);  // Detach the servo
}

// Servo control functions
void moveServo(int channel, int position) {
  // Convert angle (0-180) to pulse width
  uint16_t pulseWidth = map(position, 0, 180, 150, 600);
  servoDriver.setPWM(channel, 0, pulseWidth);
  delay(50);  // Increase delay to allow servo to move properly
}

void moveArmToScanPosition() {
  Serial.println("Moving arm to scan position");
  moveServo(BASE_CHANNEL, SCAN_BASE);
  moveServo(SHOULDER_CHANNEL, SCAN_SHOULDER);
  moveServo(ELBOW_CHANNEL, SCAN_ELBOW);
  moveServo(WRIST_ROT_CHANNEL, SCAN_WRIST_ROT);
  moveServo(WRIST_UD_CHANNEL, SCAN_WRIST_UD);
  moveServo(GRIPPER_CHANNEL, SCAN_GRIPPER);
  delay(1500);  // Increased wait time for servos to reach position
  
  // Detach servos after positioning
  detachServo(GRIPPER_CHANNEL);
  detachServo(WRIST_ROT_CHANNEL);
  detachServo(WRIST_UD_CHANNEL);
  
  Serial.println("Scan position complete");
}

// The epic pickup sequence - this took DAYS to get right!
void moveArmToPickupPosition() {
  Serial.println("Starting arm pickup position sequence");
  
  // Start with gripper wide open so we don't hit anything
  Serial.println("Preparing gripper - opening wide");
  moveServo(GRIPPER_CHANNEL, SCAN_GRIPPER);  // Open gripper fully
  delay(500);
  
  // First set the arm's base rotation
  Serial.println("Moving BASE to position " + String(PICKUP_BASE));
  moveServo(BASE_CHANNEL, PICKUP_BASE);
  
  // Move to a position slightly above the target first
  Serial.println("Moving to pre-pickup position (higher approach)");
  // This two-stage approach is much gentler on the objects
  int prePickupShoulder = PICKUP_SHOULDER + 15;
  int prePickupElbow = PICKUP_ELBOW - 10;
  
  moveServo(SHOULDER_CHANNEL, prePickupShoulder);
  moveServo(ELBOW_CHANNEL, prePickupElbow);
  
  // Set the wrist angle
  Serial.println("Moving WRIST_ROT to position " + String(PICKUP_WRIST_ROT));
  moveServo(WRIST_ROT_CHANNEL, PICKUP_WRIST_ROT);
  
  // Prepare the wrist up/down angle
  Serial.println("Preparing wrist angle");
  moveServo(WRIST_UD_CHANNEL, PICKUP_WRIST_UD);
  
  // Give time for all this to happen
  delay(1200);
  
  // Now the cool part - gently lower the arm into position
  Serial.println("Gently lowering arm to pickup position");
  
  // Lower the shoulder in small steps for smoother motion
  for (int pos = prePickupShoulder; pos >= PICKUP_SHOULDER; pos -= 3) {
    moveServo(SHOULDER_CHANNEL, pos);
    delay(100);  // Small delay between steps for smooth motion
  }
  
  // Fine tune the elbow angle
  for (int pos = prePickupElbow; pos <= PICKUP_ELBOW; pos += 2) {
    moveServo(ELBOW_CHANNEL, pos);
    delay(80);  // Small delay between increments
  }
  
  // Let things stabilize before gripping
  Serial.println("Final position stabilizing");
  delay(800);
  
  // Close the gripper gradually - this is way better than snapping shut
  Serial.println("Gently closing gripper");
  int startGripperPos = SCAN_GRIPPER;
  int endGripperPos = PICKUP_GRIPPER;
  int steps = 10;  // Number of increments
  int increment = (startGripperPos - endGripperPos) / steps;
  
  for (int i = 0; i <= steps; i++) {
    int pos = startGripperPos - (i * increment);
    moveServo(GRIPPER_CHANNEL, pos);
    delay(120);  // Slower closure for gentler grip
  }
  
  // Final squeeze to make sure we got it
  moveServo(GRIPPER_CHANNEL, PICKUP_GRIPPER);
  delay(500);
  
  // Extra tightening pulse - this really helps with keeping the grip solid
  moveServo(GRIPPER_CHANNEL, PICKUP_GRIPPER - 2);
  delay(200);
  moveServo(GRIPPER_CHANNEL, PICKUP_GRIPPER);
  delay(200);
  
  // Power down the servos we don't need anymore to save power and reduce jitter
  // But KEEP the gripper powered or we'll drop the object!
  Serial.println("Detaching most servos but keeping gripper powered");
  detachServo(WRIST_ROT_CHANNEL);
  detachServo(WRIST_UD_CHANNEL);
  // DO NOT detach gripper: detachServo(GRIPPER_CHANNEL);
  
  Serial.println("Pickup position sequence complete");
}

// Move to the carrying position while holding an object
void moveArmToCarryPosition() {
  Serial.println("Moving arm to carry position");
  
  moveServo(BASE_CHANNEL, CARRY_BASE);
  moveServo(SHOULDER_CHANNEL, CARRY_SHOULDER);
  
  // Special case for bin approach
  if (currentState == APPROACH && currentTarget == BIN_TARGET) {
    // Need a different elbow angle when approaching bins to avoid collisions
    moveServo(ELBOW_CHANNEL, FORWARD_ELBOW);
    Serial.println("Using forward elbow position (60°) for approaching after ArUco");
  } else {
    // Standard carrying position
    moveServo(ELBOW_CHANNEL, CARRY_ELBOW);
  }
  
  moveServo(WRIST_ROT_CHANNEL, CARRY_WRIST_ROT);
  moveServo(WRIST_UD_CHANNEL, CARRY_WRIST_UD);
  
  // Make absolutely sure we don't drop the object!
  moveServo(GRIPPER_CHANNEL, CARRY_GRIPPER);
  delay(1500);  // Give plenty of time for the servos to get there
  
  // Extra tightening pulse - we REALLY don't want to drop it
  moveServo(GRIPPER_CHANNEL, CARRY_GRIPPER - 2);
  delay(200);
  moveServo(GRIPPER_CHANNEL, CARRY_GRIPPER);
  delay(200);
  
  // Save power by turning off unused servos
  detachServo(WRIST_ROT_CHANNEL);
  detachServo(WRIST_UD_CHANNEL);
  // DO NOT detach gripper: detachServo(GRIPPER_CHANNEL);
  
  Serial.println("Carry position complete - maintaining maximum grip force");
}

// The dropoff sequence when we've reached a bin
void moveArmToDropoffPosition() {
  Serial.println("Moving arm to dropoff position");
  
  // Position the arm over the bin
  moveServo(BASE_CHANNEL, DROPOFF_BASE);
  moveServo(SHOULDER_CHANNEL, DROPOFF_SHOULDER);
  moveServo(ELBOW_CHANNEL, DROPOFF_ELBOW);
  moveServo(WRIST_ROT_CHANNEL, DROPOFF_WRIST_ROT);
  moveServo(WRIST_UD_CHANNEL, DROPOFF_WRIST_UD);
  delay(1500);  // Wait for the arm to get into position
  
  // Finally release our grip and drop the object into the bin
  Serial.println("Releasing object...");
  moveServo(GRIPPER_CHANNEL, DROPOFF_GRIPPER);  // Open gripper
  delay(800);   // Give it time to fully open
  
  // Power down all servos now that we're done
  detachServo(GRIPPER_CHANNEL);
  detachServo(WRIST_ROT_CHANNEL);
  detachServo(WRIST_UD_CHANNEL);
  
  Serial.println("Dropoff position complete - object released");
}

// UTILITY FUNCTIONS
// Function to print state changes for debugging
void reportStateChange() {
  if (currentState != lastReportedState) {
    Serial.print("STATE CHANGE: ");
    Serial.print(getStateName(lastReportedState));
    Serial.print(" => ");
    Serial.println(getStateName(currentState));
    
    lastReportedState = currentState;
  }
}

// Get state name as string for debugging
String getStateName(RobotState state) {
  switch(state) {
    case IDLE: return "IDLE";
    case SEARCH: return "SEARCH";
    case ALIGN_ROUGH: return "ALIGN_ROUGH";
    case ALIGN_FINE: return "ALIGN_FINE";
    case APPROACH: return "APPROACH";
    case ACTION: return "ACTION";
    case TRANSITION: return "TRANSITION";
    case COMPLETE: return "COMPLETE";
    default: return "UNKNOWN";
  }
}

// Send status messages to the app
void sendStatusMessage() {
  boolean canSendStatusMessage = (millis() - lastStatusMessageTime) > 500;
  
  if (canSendStatusMessage) {
    String message;
    
    if (currentTarget == OBJECT) {
      // Messages for object target
      switch (currentState) {
        case SEARCH: message = "SEARCHING?"; break;
        case ALIGN_ROUGH:
        case ALIGN_FINE: message = "OBJECT_FOUND?"; break;
        case APPROACH: message = "APPROACHING?"; break;
        case ACTION: message = "GRABBING?"; break;
        case TRANSITION: message = "CARRYING?"; break;
        default: message = "WAITING?"; break;
      }
    } else {
      // Messages for bin target
      switch (currentState) {
        case SEARCH: message = "BIN_SEARCH?"; break;
        case ALIGN_ROUGH:
        case ALIGN_FINE: message = "BIN_FOUND?"; break;
        case APPROACH: message = "BIN_APPROACH?"; break;
        case ACTION: message = "DROPPING?"; break;
        case COMPLETE: message = "COMPLETED?"; break;
        default: message = "WAITING?"; break;
      }
    }
    
    Serial1.println(message);
    lastStatusMessageTime = millis();
  }
}
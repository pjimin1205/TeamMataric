#include <RPC.h>
#include <Arduino.h>//include for PlatformIO Ide
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
// --------------------------------------- Constant Vars -------------------------------------------------------------
#define INCHES_TO_CM 2.54 //conversion factor from inches to centimeters
#define TWO_FEET_IN_STEPS 1848 //number of steps to move robot forward 2 feet
#define WIDTH_OF_BOT_CM 21.2 //width of robot in centimeters, distance between wheels center to center
#define WHEEL_DIAM_CM 8.4 // wheel diameter in centimeters
#define STEPS_PER_ROT 800 //number of steps per wheel rotation
#define CM_PER_ROTATION 26.39 //circumference of wheel in centimeters
#define CM_TO_STEPS_CONV (STEPS_PER_ROT/(float)CM_PER_ROTATION) //conversion factor from centimeters to steps
#define ENCODER_TICKS_PER_ROTATION 20 //number of encoder ticks per wheel rotation
#define CM_PER_FOOT 30.48 // number of centimeters in a foot
// ------------------------------------- LED's -------------------------------------------------------------------------
#define redLED 5            //red LED for displaying states
#define greenLED 6            //green LED for displaying states
#define grnLED 6            //green LED for displaying states
#define yellowLED 7            //yellow LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED
int leds[3] = {5,6,7};      //array of LED pin numbers
// ------------------------------------- Motor set up -------------------------------------------------------------------
#define stepperEnable 48    //stepper enable pin on stepStick 
#define rtStepPin 50 //right stepper motor step pin 
#define rtDirPin 51  // right stepper motor direction pin 
#define ltStepPin 52 //left stepper motor step pin 
#define ltDirPin 53  //left stepper motor direction pin

// --------------------------------- goToGoal Initial Vars -------------------------------------------------------------------
// Odometry Variables
float robotX = 0.0;     // cm
float robotY = 0.0;     // cm
float robotTheta = 0.0; // Radians (0 is facing 'East' or forward)

long lastLeftSteps = 0;
long lastRightSteps = 0;

// Goal Variables
float goalX = 30*9;    // Target 9 feet ahead
float goalY = -30*3;    // Target 3 foot to the left
unsigned long lastOdoPrint = 0;

// Goal Flag
bool goalSet = true;
bool goalReached = false;


AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin);//create instance of right stepper motor object (2 driver pins, low to high transition step pin 52, direction input pin 53 (high means forward)
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);//create instance of left stepper motor object (2 driver pins, step pin 50, direction input pin 51)
MultiStepper steppers;//create instance to control multiple steppers at the same time

#define stepperEnTrue false //variable for enabling stepper motor
#define stepperEnFalse true //variable for disabling stepper motor
#define max_speed 1500 //maximum stepper motor speed
#define max_accel 10000 //maximum motor acceleration
#define base_speed 500

int pauseTime = 2500;   //time before robot moves
int stepTime = 500;     //delay time between high and low on step pin
int wait_time = 1000;   //delay for printing data

int minIntervalAngle = 13.79; // degrees 
int leftSpd = 500; // default speed for left and right motors in septs per second
int rightSpd = 500;

int leftSonarDist = 1000; // default left sonar distance
int rightSonarDist = 1000; // default right sonar distance

int targetFollowDist = 10; //cm
int minimumFollowingDist = 5; //cm
int maximumStoppingDist = 25; //cm

int min_deadband_cm = 10;
int max_deadband_cm = 15;

// Force Constants
const int MAX_FORCE = 100;
const float K_REPULSIVE = 200.0;

// Force Vector values
float forceVectorx = 0;
float forceVectory = 0;
float forceVectorAngle = 0;

struct MotorCommand{
  int leftSpeed;
  int rightSpeed;
  bool active;
};

// ------------------------------------- Sensors -------------------------------------------------------------------
const float ANGLE_LEFT_SONAR = 45.0;     // Left sonar at 45 degrees
const float ANGLE_RIGHT_SONAR = -45.0;   // Right sonar at -45 degrees
const float ANGLE_FRONT_LIDAR = 0.0;     // Front LIDAR at 0 degrees
const float ANGLE_BACK_LIDAR = 180.0;    // Back LIDAR at 180 degrees
const float ANGLE_LEFT_LIDAR = 90.0;     // Left LIDAR at 90 degrees
const float ANGLE_RIGHT_LIDAR = -90.0;   // Right LIDAR at -90 degrees
const float ANGLE_BACK_LEFT_SONAR = 90.0;
const float ANGLE_BACK_RIGHT_SONAR = -90.0;

bool sensorsReady = false; // Flag to indicate if sensors have been initialized

struct SensorPacket {
    int frontLidar;
    int backLidar;
    int leftLidar;
    int rightLidar;
    int frontLeftSonar;
    int frontRightSonar;
    int backLeftSonar;
    int backRightSonar;
    int photoLeft;
    int photoRight;
    int encoderLeft;
    int encoderRight;
    int huskyLensY;
    int huskyLensX;
    int huskyLensWidth;
    int huskyLensHeight;
    // This macro tells the RPC/MsgPack layer how to serialize fields
    MSGPACK_DEFINE_ARRAY(frontLidar, backLidar, leftLidar, rightLidar, frontLeftSonar, frontRightSonar, backLeftSonar, backRightSonar, photoLeft, photoRight, encoderLeft, encoderRight, huskyLensX, huskyLensY, huskyLensWidth, huskyLensHeight);
};
unsigned long lastSensorRequest = 0;
const long sensorInterval = 100; // Query sensors every 100 ms
SensorPacket sensorData;
bool printSenRead = false; // for debug

/*
  updateSensorData()
  -------------------
  Throttled RPC call to request sensor data from the M4 core and
  unpack it into `sensorData`. Optionally prints sensor readings
  when `printSenRead` is enabled.
  Side-effects: modifies global `sensorData`.
*/
void updateSensorData(){
  // Only talk to the M4 every 100ms
  if (millis() - lastSensorRequest >= sensorInterval) {
    lastSensorRequest = millis();
    auto res = RPC.call("getSensorData");
    sensorData = res.as<SensorPacket>();

    if(printSenRead){
      Serial.print("F: "); Serial.print(sensorData.frontLidar);
      Serial.print(" | B: "); Serial.print(sensorData.backLidar);
      Serial.print(" | L: "); Serial.print(sensorData.leftLidar);
      Serial.print(" | R: "); Serial.print(sensorData.rightLidar);
      Serial.print(" | LSonar: "); Serial.print(sensorData.frontLeftSonar);
      Serial.print(" | RSonar: "); Serial.print(sensorData.frontRightSonar);
      Serial.print(" | BLSonar: "); Serial.print(sensorData.backLeftSonar);
      Serial.print(" | BRSonar: "); Serial.print(sensorData.backRightSonar);
      Serial.println();
    }
  }
}

void updateOdometry() {
    // Get current absolute positions from the AccelStepper objects
    long currLeft = stepperLeft.currentPosition();
    long currRight = stepperRight.currentPosition();

    // 1. Calculate change in steps
    long dLeftSteps = currLeft - lastLeftSteps;
    long dRightSteps = currRight - lastRightSteps;

    // 2. Convert steps to distance (cm)
    // Note: Use (float) to ensure decimal precision
    float dLeftCM = -(dLeftSteps / (float)CM_TO_STEPS_CONV);
    float dRightCM = -(dRightSteps / (float)CM_TO_STEPS_CONV);

    // 3. Differential Drive Kinematics
    float dCenter = (dLeftCM + dRightCM) / 2.0;
    float dPhi = (dRightCM - dLeftCM) / WIDTH_OF_BOT_CM; // Change in heading

    // 4. Update Global Pose
    robotX += dCenter * cos(robotTheta + (dPhi / 2.0));
    robotY += dCenter * sin(robotTheta + (dPhi / 2.0));
    robotTheta += dPhi;

    // Keep theta between -PI and PI
    if (robotTheta > PI) robotTheta -= TWO_PI;
    if (robotTheta < -PI) robotTheta += TWO_PI;

    // Save state for next loop
    lastLeftSteps = currLeft;
    lastRightSteps = currRight;

    if (millis() - lastOdoPrint > 500) { // Print every 500ms
        lastOdoPrint = millis();
        Serial.print("Pose: X=");
        Serial.print(robotX);
        Serial.print(" Y=");
        Serial.print(robotY);
        Serial.print(" Theta=");
        Serial.println(robotTheta * 180 / PI); // Convert to degrees for readability
    }
}

float wrapAngle(float a) {
  while (a > PI) a -= TWO_PI;
  while (a < -PI) a += TWO_PI;
  return a;
}

enum NavState { // state machine, only affects goal/navigation behaviors
  LEFT_WALL,
  RIGHT_WALL,
  CENTER,
  AVOID,
  RANDOM_WANDER,
  GO_TO_GOAL
};

NavState currState = RANDOM_WANDER;  // Change this to switch modes
int wallLostTimestamp = 0;
int stateDelay = 500; // 5000 -> 0 ms, delays state for 5000 ms
int preventTime = -5000;

int randomWanderTimer = 0;
# define randomWanderInterval 1000

/*
  Sets the current state between CENTER, LEFT_WALL, RIGHT_WALL, RANDOM_WANDER and GO_TO_GOAL
*/
void updateNavState_wall_follow() {   
    if (sensorData.backLidar < maximumStoppingDist && sensorData.backLidar > 0
    || millis() - preventTime < stateDelay){
      // avoidFlag == true;
      // avoidStartTime = millis();   
      return;// Stay in current state but let avoidance handle it
    }
  //Serial.println(currState);
  bool leftWallPresent = (sensorData.rightLidar > 0 && sensorData.rightLidar < 30);
  bool rightWallPresent = (sensorData.leftLidar > 0 && sensorData.leftLidar < 30);
 
  if(leftWallPresent && rightWallPresent){ // sees both walls
    currState = CENTER;
    wallLostTimestamp = 0;
  }
  else if(leftWallPresent){ // sees left wall
    currState = LEFT_WALL;
    wallLostTimestamp = 0;
  }
  else if(rightWallPresent){ // sees right wall
    currState = RIGHT_WALL;
    wallLostTimestamp = 0;
  }
  else{ // doesn't see any wall
    if(wallLostTimestamp == 0){
      wallLostTimestamp = millis();
    }
    else if(millis() - wallLostTimestamp > 3000 && !goalSet){ // for an extended period of time and if there is no goal set
      // if no walls for 3 seconds, go to random wander
      currState = RANDOM_WANDER;
    } else if(goalSet){ // if there is a goal set, go to goal
      currState = GO_TO_GOAL;
    }
  }
}

int pathIndex = 0; // update path index once a move has been completed

bool pathActionActive = false;

// Odometry targets
float targetTheta = 0;
float targetDistance = 0;

// Starting pose for actions
float actionStartX = 0;
float actionStartY = 0;
float actionStartTheta = 0;

// Tunables - may be able to delete later
const float TURN_TOL = 3.0 * DEG_TO_RAD;   // ~3 deg
const float DIST_TOL = 2.0;                // cm
const float CELL_FORWARD_DIST = 45.0;      // hallway length (~18 in)

enum NavPathState { LEFT, RIGHT, FORWARD, START, TERMINATE};
NavPathState currPathState = START;
char path[] = "SFFRFFFRFT";

/*
  Sets the current path state between turn LEFT, RIGHT, FORWARD, START, TERMINATE
  Shouldn't need to account for blocked state because path accounts for that already.
*/
void updateNavState_path() {
  if (pathActionActive) return; // wait until current action finishes

  char cmd = path[pathIndex];

  switch (cmd) {
    case 'S':
      currPathState = START;
      break;

    case 'F':
      currPathState = FORWARD;
      break;

    case 'L':
      currPathState = LEFT;
      break;

    case 'R':
      currPathState = RIGHT;
      break;

    case 'T':
      currPathState = TERMINATE;
      break;

    default:
      currPathState = TERMINATE;
      break;
  }
}


// ------------------------------------- START OF FUNCTIONS ------------------------------------------------------------------
void init_stepper(){
  pinMode(rtStepPin, OUTPUT);//sets pin as output
  pinMode(rtDirPin, OUTPUT);//sets pin as output
  pinMode(ltStepPin, OUTPUT);//sets pin as output
  pinMode(ltDirPin, OUTPUT);//sets pin as output
  pinMode(stepperEnable, OUTPUT);//sets pin as output
  digitalWrite(stepperEnable, stepperEnFalse);//turns off the stepper motor driver
  pinMode(enableLED, OUTPUT);//set enable LED as output
  digitalWrite(enableLED, LOW);//turn off enable LED
  pinMode(redLED, OUTPUT);//set red LED as output
  pinMode(grnLED, OUTPUT);//set green LED as output
  pinMode(ylwLED, OUTPUT);//set yellow LED as output
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED
  delay(pauseTime / 5); //wait 0.5 seconds
  digitalWrite(redLED, LOW);//turn off red LED
  digitalWrite(ylwLED, LOW);//turn off yellow LED
  digitalWrite(grnLED, LOW);//turn off green LED

  stepperRight.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperRight.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  stepperLeft.setMaxSpeed(max_speed);//set the maximum permitted speed limited by processor and clock speed, no greater than 4000 steps/sec on Arduino
  stepperLeft.setAcceleration(max_accel);//set desired acceleration in steps/s^2
  steppers.addStepper(stepperRight);//add right motor to MultiStepper
  steppers.addStepper(stepperLeft);//add left motor to MultiStepper
  digitalWrite(stepperEnable, stepperEnTrue);//turns on the stepper motor driver
  digitalWrite(enableLED, HIGH);//turn on enable LED
}

// -----------------------------------------Layer 3 ------------------------------------------------------------------------
int randLeftSpeed = base_speed;
int randRightSpeed = base_speed;
MotorCommand randomWander(){
  digitalWrite(redLED, HIGH);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED

  if(millis() - randomWanderTimer > randomWanderInterval){
    randomWanderTimer = millis();
    randLeftSpeed = random(100, 1000);
    randRightSpeed = random(100, 1000);
  }
  
  return {randLeftSpeed, randRightSpeed, true};
}

MotorCommand goToGoal(float targetX, float targetY) {
    MotorCommand cmd = {0, 0, true};

    digitalWrite(redLED, HIGH);//turn on red LED
    digitalWrite(ylwLED, HIGH);//turn on yellow LED
    digitalWrite(grnLED, HIGH);//turn on green LED


    // 1. Calculate vector to goal
    float dx = targetX - robotX;
    float dy = targetY - robotY;
    float distanceToGoal = sqrt(dx * dx + dy * dy);
    float angleToGoal = atan2(dy, dx);

    // 2. Calculate heading error
    float alpha = angleToGoal - robotTheta;
    // Normalize alpha to (-PI, PI)
    while (alpha > PI) alpha -= TWO_PI;
    while (alpha < -PI) alpha += TWO_PI;
    Serial.println(distanceToGoal);
    // 3. Stop condition
    if (distanceToGoal < 1) { // Within 5cm
        Serial.println("Goal Reached");
        return {0, 0, false}; 
    }

    // 4. Control Gains (Tweak these!)
    float K_linear = 10;  // Speed towards goal
    float K_angular = 400; // Speed of turning

    float v = K_linear * distanceToGoal;
    float w = K_angular * alpha;

    // Limit maximums to your base_speed
    v = constrain(v, -base_speed, base_speed);
    
    cmd.leftSpeed = v - w;
    cmd.rightSpeed = v + w;
    cmd.active = true;

    return cmd;
}
// -----------------------------------------Layer 2 ------------------------------------------------------------------------
// example
// collection of layer 1 movements, this is layer 2 because it has logic.
MotorCommand moveBehavior(){
  switch (currState){
    case LEFT_WALL:
      return followLeftWallPD();
    case RIGHT_WALL:
      return followRightWallPD();
    case CENTER:
      return followCenter();
    case GO_TO_GOAL:
      return goToGoal(goalX, goalY);
    case RANDOM_WANDER:
      return randomWander();
    default:
      return {0, 0, false};
  }
}

MotorCommand movePathBehavior(){
  switch (currState){
    case LEFT:
      return turnLeft();
    case RIGHT:
      return turnRight();
    case FORWARD:
      return goForward();
    case START:
      return startPath();
    case TERMINATE:
      return terminatePath();
    default:
      return {0, 0, false};
  }
}

MotorCommand followCenter(){
  MotorCommand cmd = {base_speed, base_speed, true};

  // LED Requirement: Turn on ALL 3 LEDs when following center [cite: 244, 269]
  digitalWrite(redLED, HIGH);
  digitalWrite(ylwLED, LOW);
  digitalWrite(grnLED, LOW);

  // Distances (Adjust sensor mapping based on your specific robot setup)
  // Based on your previous code: rightLidar is on the left, leftLidar is on the right
  float leftDist = sensorData.rightLidar; 
  float rightDist = sensorData.leftLidar;

  // Calculate the error: Difference between the two distances [cite: 30]
  // A positive error means we are closer to the right wall
  float error = leftDist - rightDist;

  // Proportional gain (Suggested between 1 and 10) [cite: 44, 199]
  const float Kp_center = 8.0; 
  float turn = Kp_center * error;

  // Constrain turn to prevent overwhelming base speed [cite: 31]
  turn = constrain(turn, -125, 125);

  // Apply steering logic
  cmd.leftSpeed  = base_speed + turn;
  cmd.rightSpeed = base_speed - turn;

  return cmd;
}
// -----------------------------------------Layer 1 ------------------------------------------------------------------------
// PD Control (uses angle to approximate behavior instead of time)
MotorCommand followRightWallPD() {
  MotorCommand cmd = {base_speed, base_speed, true};
  digitalWrite(redLED, LOW);//turn on red LED
  digitalWrite(ylwLED, HIGH);//turn on yellow LED
  digitalWrite(grnLED, LOW);//turn on green LED

  const float desiredDist = 12.0; // cm
  const float Kp = 10.0;          // distance gain: 10-15
  const float Kd = 80.0;          // angle gain (radians!): 50-100

  float leftDist = sensorData.leftLidar;
  float backLeft = sensorData.backLeftSonar;

  float distError = desiredDist - leftDist;

  // Angle error (already computed in your bang code)
  float height = leftDist - backLeft; // if negative, needs to turn (-) angle, if positive, need to turn (+) angle
  float base = 8.0; // cm between sensors
  float angleError = atan2(height, base); // radians

  float turn = (Kp * distError) + (Kd * angleError);
  turn = constrain(turn, -125, 125);

  cmd.leftSpeed  = base_speed + turn;
  cmd.rightSpeed = base_speed - turn;

  // if we lost the wall, turn left
  if (leftDist <= 0 || leftDist > 50.0) {
    // Perform a circle motion: Left wheel slow, Right wheel fast 
    // to arc back toward where the wall should be.
    cmd.leftSpeed  = base_speed * 0.4; 
    cmd.rightSpeed = base_speed * 1.2;
    return cmd; 
  }
  return cmd;
}
// PD Control (uses angle to approximate behavior instead of time)
MotorCommand followLeftWallPD() {
  MotorCommand cmd = {base_speed, base_speed, true};
  
  digitalWrite(redLED, LOW);//turn on red LED
  digitalWrite(ylwLED, LOW);//turn on yellow LED
  digitalWrite(grnLED, HIGH);//turn on green LED

  const float desiredDist = 12.0; // cm
  const float Kp = 10.0;          // distance gain: 10-15
  const float Kd = 80.0;          // angle gain (radians!): 50-100

  float leftDist = sensorData.rightLidar;
  float backLeft = sensorData.backRightSonar;

  float distError = desiredDist - leftDist;

  // Angle error (already computed in your bang code)
  float height = leftDist - backLeft; // if negative, needs to turn (-) angle, if positive, need to turn (+) angle
  float base = 8.0; // cm between sensors
  float angleError = atan2(height, base); // radians

  float turn = (Kp * distError) + (Kd * angleError);
  turn = constrain(turn, -125, 125);

  cmd.leftSpeed  = base_speed - turn;
  cmd.rightSpeed = base_speed + turn;

  // if we lost the wall, turn left
  if (leftDist <= 0 || leftDist > 50.0) {
    // Perform a circle motion: Left wheel slow, Right wheel fast 
    // to arc back toward where the wall should be.
    cmd.leftSpeed  = base_speed * 1.2; 
    cmd.rightSpeed = base_speed * 0.4;
    return cmd; 
  }

  return cmd;
}
// if robot detects object too close, closer than deadband, use force vectors to move away
MotorCommand avoidObstacle(){
  MotorCommand cmd = {0, 0, false};
  int sensorThreshold = 15;
  int avoidTurnSpeed = 400;

  int distCenter = sensorData.backLidar;
  // if current time - start time > 1000
  // avoidFlag = false;
  int distL = sensorData.rightLidar;
  int distR = sensorData.leftLidar;

  // 1. SENSOR CHECK: Is there an obstacle RIGHT NOW?
  bool obstacleDetected = (distCenter > 0 && distCenter < sensorThreshold);

  // 2. TIMER UPDATE: If we see something, reset the "Safe To Stop" clock
  if (obstacleDetected) {
    preventTime = millis();
  }

  // 3. STATE CHECK: Are we currently avoiding (either seeing it or recently saw it)?
  if (millis() - preventTime < stateDelay) {    
    cmd.active = true;
    digitalWrite(enableLED, HIGH);

    // Steering logic (keep your current logic)
    if (distL < distR) {
      cmd.leftSpeed = base_speed + avoidTurnSpeed;
      cmd.rightSpeed = base_speed - avoidTurnSpeed;
    } else {
      cmd.leftSpeed = base_speed - avoidTurnSpeed;
      cmd.rightSpeed = base_speed + avoidTurnSpeed;
    }
  } else {
    digitalWrite(enableLED, LOW);
  }

  return cmd;
}
MotorCommand turnLeft() {
  MotorCommand cmd = {0, 0, true};

  if (!pathActionActive) {
    pathActionActive = true;
    actionStartTheta = robotTheta;
    targetTheta = wrapAngle(robotTheta + PI/2);
  }

  float err = wrapAngle(targetTheta - robotTheta);

  if (fabs(err) < TURN_TOL) {
    pathActionActive = false;
    pathIndex++;
    return {0, 0, false};
  }

  int turnSpeed = constrain(400 * err, -600, 600); // P control
  cmd.leftSpeed  = -turnSpeed;
  cmd.rightSpeed =  turnSpeed;

  return cmd;
}

MotorCommand turnRight() {
  MotorCommand cmd = {0, 0, true};

  if (!pathActionActive) {
    pathActionActive = true;
    actionStartTheta = robotTheta;
    targetTheta = wrapAngle(robotTheta - PI/2);
  }

  float err = wrapAngle(targetTheta - robotTheta);

  if (fabs(err) < TURN_TOL) {
    pathActionActive = false;
    pathIndex++;
    return {0, 0, false};
  }

  int turnSpeed = constrain(400 * err, -600, 600);
  cmd.leftSpeed  = -turnSpeed;
  cmd.rightSpeed =  turnSpeed;

  return cmd;
}

MotorCommand goForward() {
  MotorCommand cmd = {base_speed, base_speed, true};

  if (!pathActionActive) {
    pathActionActive = true;
    actionStartX = robotX;
    actionStartY = robotY;
  }

  float dx = robotX - actionStartX;
  float dy = robotY - actionStartY;
  float dist = sqrt(dx*dx + dy*dy);

  if (dist >= CELL_FORWARD_DIST) {
    pathActionActive = false;
    pathIndex++;
    return {0, 0, false};
  }

  return cmd;
}

MotorCommand startPath() {
  pathActionActive = false;
  pathIndex++;
  return {0, 0, false};
}
MotorCommand terminatePath() {
  digitalWrite(redLED, HIGH);
  digitalWrite(ylwLED, HIGH);
  digitalWrite(grnLED, HIGH);

  return {0, 0, false};
}

// -----------------------------------------Layer 0 ------------------------------------------------------------------------
// robot detects object too close, stops the robot
MotorCommand collide(){
  MotorCommand cmd = {0, 0, false};
  int wallDetectDist = 10;
  float speedMod = 2;
  unsigned long collisionRecoveryTimer = 0;
  const int retreatDuration = 4000; // Move forward for 1 seconds
  
  // 1. Check for immediate collision while reversing
  if (sensorData.backLidar > 0 && sensorData.backLidar < 8) {
    cmd.leftSpeed = 0;
    cmd.rightSpeed = 0;
    cmd.active = true;
    // collisionRecoveryTimer = millis(); // Start/Reset retreat timer
  }
  return cmd;
}
// -------------------------------------- Main --------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  init_stepper(); //set up stepper motor
  RPC.begin(); 
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDB, LOW); // Blue ON
  stepperLeft.setSpeed(leftSpd);
  stepperRight.setSpeed(rightSpd);
  Serial.println("M7: Online. Requesting data...");
  //initial sensor
  delay(2000); 

  // Wait until we get a reading that isn't zero from a known sensor
  Serial.println("Waiting for valid sensor data...");
  while (!sensorsReady) {
    updateSensorData();
    if (sensorData.backLidar > 0 || sensorData.frontLidar > 0) {
      sensorsReady = true;
      Serial.println("Sensors Online!");
    }
    delay(100); 
  }
}

void loop() {
  updateSensorData();
  updateNavState_path();
  updateNavState_wall_follow();
  updateOdometry();

  MotorCommand cmd = {0, 0, false};
  MotorCommand c;
  MotorCommand p;
  
  // 1. DEFAULT: If no goal, default to forward wander. If goal, default to stop.
  if(!goalSet) {
    cmd = {base_speed, base_speed, true}; 
  } else {
    cmd = {0, 0, false};
  }
  
  // --- LAYER 0: COLLISION (Highest Priority) ---
  c = collide();
  if(c.active){ cmd = c; goto APPLY; }

  // --- LAYER 1: OBSTACLE AVOIDANCE ---
  c = avoidObstacle();
  if(c.active){ cmd = c; goto APPLY; }

  // --- LAYER 2: GO TO GOAL ---
  // If a goal is set, try to do this FIRST.
  if(goalSet) {
    c = goToGoal(goalX, goalY);
    if(c.active) {
      cmd = c;
      goto APPLY; // "Break away" from walls to head to goal!
    }
  }

  // --- LAYER 3: NAVIGATION (Wall Following / Center) ---
  // If no goal is set, or if goToGoal is inactive, follow walls.
  // c = moveBehavior();
  if(c.active) {
    cmd = c;
    goto APPLY;
  }
  p = movePathBehavior();
  if (p.active || currPathState == TERMINATE) {
    cmd = p;
    goto APPLY;
  }

  APPLY:
    stepperLeft.setSpeed(-cmd.leftSpeed);
    stepperRight.setSpeed(-cmd.rightSpeed);
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
}

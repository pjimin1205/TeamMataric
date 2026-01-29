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
#define CM_TO_STEPS_CONV STEPS_PER_ROT/CM_PER_ROTATION //conversion factor from centimeters to steps
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
    // This line tells the RPC library how to pack the data
    MSGPACK_DEFINE_ARRAY(frontLidar, backLidar, leftLidar, rightLidar, frontLeftSonar, frontRightSonar, backLeftSonar, backRightSonar);
};
unsigned long lastSensorRequest = 0;
const long sensorInterval = 100; // Query sensors every 100 ms
SensorPacket sensorData;
bool printSenRead = true; // for debug

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

void updateNavState() {   
    if (sensorData.backLidar < maximumStoppingDist && sensorData.backLidar > 0
    || millis() - preventTime < stateDelay){
      // avoidFlag == true;
      // avoidStartTime = millis();   
      return;// Stay in current state but let avoidance handle it
    }
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
    else if(millis() - wallLostTimestamp > 3000){ // for an extended period of time
      currState = RANDOM_WANDER;
    }
  }
  // */
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
    // case GO_TO_GOAL:
    //   return goToGoal();
    case RANDOM_WANDER:
      return randomWander();
    default:
      return {base_speed, base_speed, true};
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
// follow right wall
MotorCommand followRightWallBang(){
  MotorCommand cmd = {base_speed, base_speed, true};

  int backLeftSonarDist = sensorData.backLeftSonar; // CM
  int leftLidarDist = sensorData.leftLidar; // CM
  int frontLeftSonarDist = sensorData.frontLeftSonar*sin(ANGLE_LEFT_SONAR*PI/180); // distance to the left, from the front sensor

  // if too far from left deadband
  if(leftLidarDist > max_deadband_cm){
    Serial.print("dist: ");
    Serial.print(leftLidarDist);
    Serial.println(" | too far");
    cmd.rightSpeed = base_speed + 125;
  }
  // if too close to left
  else if(leftLidarDist < min_deadband_cm && leftLidarDist > 0){
    Serial.print("dist: ");
    Serial.print(leftLidarDist);
    Serial.println(" | too close");
    cmd.leftSpeed = base_speed + 125;
  }
  else{
    Serial.print("dist: ");
    Serial.print(leftLidarDist);
    Serial.println(" | gooooood");
  } // in left deadband
  // if sensors don't detect left wall/object, then it's in RANDOM WANDER state
  return cmd;
}
// Proportional Control 
MotorCommand followRightWallP(){
  MotorCommand cmd = {base_speed, base_speed, true};

  const float desiredDist = 12.0;   // cm, between 10, 15 cm
  const float Kp = 10.0;             // tuning gain

  float leftLiDist = sensorData.leftLidar;
  float distError = desiredDist - leftLiDist;

  float turn = Kp * distError;
  turn = constrain(turn, -125, 125);

  cmd.leftSpeed  = base_speed + turn;
  cmd.rightSpeed = base_speed - turn;

  return cmd;
}
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
    // Serial.print("millis: ");
    // Serial.print(millis());
    // Serial.print(", preventTime: ");
    // Serial.print(preventTime);
    // Serial.print(", stateDelay: ");
    // Serial.print(stateDelay);
    // Serial.print(", bool: ");
    // Serial.println(millis() - preventTime < stateDelay);
    
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

  /*
  // 2. If we are within the retreat window, move FORWARD
  if ((millis() - collisionRecoveryTimer < retreatDuration)) {
    // To move forward, we need the opposite of our base_speed
    // will not move backward if path is blocked
    // because your APPLY section uses -cmd.speed
    if(sensorData.rightLidar < sensorData.leftLidar){
      cmd.leftSpeed = -base_speed; 
      cmd.rightSpeed = -base_speed*speedMod;
    } else if(sensorData.leftLidar < sensorData.rightLidar){
      cmd.leftSpeed = -base_speed*speedMod; 
      cmd.rightSpeed = -base_speed;
    } else{
      cmd.leftSpeed = -base_speed; 
      cmd.rightSpeed = -base_speed;
    }
    
    cmd.active = true;
    
    // Visual indicator of collision retreat
    digitalWrite(redLED, HIGH); 
  } else {
    digitalWrite(redLED, LOW);
  }
  */

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
  updateNavState();
  
  // Start with a default forward (backward-moving) command
  MotorCommand cmd = {base_speed, base_speed, true}; 
  MotorCommand c;

  // --- LAYER 0: COLLISION (Highest Priority) ---
  c = collide();
  if(c.active){
    cmd = c;
    goto APPLY; 
  }

  // --- LAYER 1: OBSTACLE AVOIDANCE ---
  c = avoidObstacle();
  if(c.active){
    cmd = c;
    goto APPLY;
  }

  // --- LAYER 2: NAVIGATION (Wall Following / Center) ---
  c = moveBehavior();
  if(c.active){
    cmd = c;
    goto APPLY;
  }

  APPLY:
    stepperLeft.setSpeed(-cmd.leftSpeed);
    stepperRight.setSpeed(-cmd.rightSpeed);
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
}

#include <RPC.h>
#include <Arduino.h>//include for PlatformIO Ide
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library
/*
  LogicNMotorsM7.ino
  -------------------
  Purpose:
    Control logic for an M7 robot using two stepper drive motors and
    multiple range sensors (LIDAR and sonar). Implements simple
    behavior modes: COLLIDE (drive forward, stop at obstacle),
    RUNAWAY (stay still and escape when approached), FOLLOW (follow
    a target), RANDOM_WANDER and two SMART modes that combine
    behaviors.

  Hardware / Pinout Summary:
    - Right stepper: step pin = `rtStepPin` (50), dir pin = `rtDirPin` (51)
    - Left  stepper: step pin = `ltStepPin` (52), dir pin = `ltDirPin` (53)
    - Stepper enable: `stepperEnable` (48)
    - State LEDs: pins 5 (red), 6 (green), 7 (yellow)

  Notes:
    - This file runs on the M7 core and requests sensor data from
      the M4 using `RPC.call("getSensorData")`.
    - Tweak the `currentMode` value or control it externally to
      change runtime behavior.
*/

// ---------- Physical constants & conversion macros ----------
#define INCHES_TO_CM 2.54 // conversion factor from inches to centimeters
#define TWO_FEET_IN_STEPS 1848 // empirical steps for ~2ft forward (project-specific)
#define WIDTH_OF_BOT_CM 21.2 // distance between wheel centers (cm)
#define WHEEL_DIAM_CM 8.4 // wheel diameter in cm
#define STEPS_PER_ROT 800 // full-step microstepping steps per wheel rotation
#define CM_PER_ROTATION 26.39 // wheel circumference in cm (measured)
#define CM_TO_STEPS_CONV STEPS_PER_ROT/CM_PER_ROTATION // cm -> steps conversion factor (float division expected)
#define ENCODER_TICKS_PER_ROTATION 20 // encoder ticks per wheel rotation (if used)
#define CM_PER_FOOT 30.48 // centimeters in a foot

// ---------- High-level behavior modes ----------
// Modes select the robot's high-level strategy for motion/obstacle handling.
enum BehaviorMode {
  COLLIDE_MODE,    // Drive forward; stop when obstacle detected
  RUNAWAY_MODE,    // Stay still; escape when obstacle approaches
  FOLLOW_MODE,     // Follow a detected object at `targetFollowDist`
  RANDOM_WANDER,   // Periodically change heading and wander
  SMART_WANDER_MODE, // FSM: wander, detect collision, avoid
  SMART_FOLLOW_MODE  // FSM: follow but respond to collisions
};
BehaviorMode currentMode = SMART_WANDER_MODE;  // Default runtime mode

// ---------- Debug / telemetry toggles ----------
// Enable serial prints for debugging sensor values and force calculations
bool printSenRead = false;
bool printForceCalc = false;

// ---------- Timing / intervals ----------
unsigned long lastSensorRequest = 0;
const long sensorInterval = 100; // Query sensors every 100 ms

unsigned long lastReorientRequest = 0; // used by wander routine
const long reorientInterval = 3000; // ms between re-orientations when wandering

// ---------- Potential field / motion control constants ----------
const int MAX_FORCE = 100;           // maximum repulsive force magnitude
const float K_REPULSIVE = 200.0;     // repulsive force gain (tune experimentally)

// Computed repulsive vector (updated each loop)
float forceVectorx = 0;
float forceVectory = 0;
float forceVectorAngle = 0; // angle of the repulsive vector (radians)

// Motion parameters
int spinSpeed = 200; // angular spin speed used for in-place turns
int maxSpeed = 500;  // forward speed for collision mode

// Simple proportional controllers for follow behavior
float KP_STEERING = 20; // steering gain
float KP_DISTANCE = 20; // distance (speed) gain

int trapDetectionModifier = 10; // small slack when checking for being trapped

// ---------- Sensor mounting angles (degrees) ----------
// These angles are used to project repulsive forces into the robot frame.
const float ANGLE_LEFT_SONAR = 45.0;     // left diagonal sonar (deg)
const float ANGLE_RIGHT_SONAR = -45.0;   // right diagonal sonar (deg)
const float ANGLE_FRONT_LIDAR = 0.0;     // front lidar (deg)
const float ANGLE_BACK_LIDAR = 180.0;    // back lidar (deg)
const float ANGLE_LEFT_LIDAR = 90.0;     // left lidar (deg)
const float ANGLE_RIGHT_LIDAR = -90.0;   // right lidar (deg)


#define redLED 5        // red state LED
#define greenLED 6      // green state LED
#define grnLED 6        // alternate green alias
#define yellowLED 7     // yellow state LED
#define ylwLED 7        // yellow alias
#define enableLED 13    // stepper enabled indicator LED
int leds[3] = {5,6,7}; // convenience array for LEDs

// ---------- Motor pin definitions ----------
#define stepperEnable 48 // stepper driver enable (active low/high dependent on board)
#define rtStepPin 50     // right stepper STEP pin
#define rtDirPin 51      // right stepper DIRECTION pin
#define ltStepPin 52     // left stepper STEP pin
#define ltDirPin 53      // left stepper DIRECTION pin


AccelStepper stepperRight(AccelStepper::DRIVER, rtStepPin, rtDirPin); // right stepper controller
AccelStepper stepperLeft(AccelStepper::DRIVER, ltStepPin, ltDirPin);  // left stepper controller
MultiStepper steppers; // coordinate blocking moves of multiple steppers

// Stepper enable logic macros (board-specific; check wiring)
#define stepperEnTrue false
#define stepperEnFalse true
#define max_speed 1500  // max allowed speed configured for AccelStepper
#define max_accel 10000 // max allowed acceleration

int pauseTime = 2500; // initial LED pause on startup (ms)
int stepTime = 500;   // step pulse timing placeholder (ms)
int wait_time = 1000; // generic wait/print delay (ms)

int minIntervalAngle = 13.79; // minimal angle interval used by navigation routines
int leftSpd = 500;  // default left speed (steps/sec)
int rightSpd = 500; // default right speed (steps/sec)

// Sonar placeholder defaults (cm)
int leftSonarDist = 1000;
int rightSonarDist = 1000;

// Follow behavior distances (cm)
int targetFollowDist = 5; // desired following distance
int minimumFollowingDist = 5; // minimum allowed following distance
int maximumStoppingDist = 10; // distance threshold considered a collision

// ---------- Sensor data structure exchanged via RPC ----------
struct SensorPacket {
  int frontLidar;
  int backLidar;
  int leftLidar;
  int rightLidar;
  int leftSonar;
  int rightSonar;
  // Tells RPC/MsgPack how to serialize fields in order
  MSGPACK_DEFINE_ARRAY(frontLidar, backLidar, leftLidar, rightLidar, leftSonar, rightSonar);
};

// Current sensor values (updated by updateSensorData())
SensorPacket sensorData;

/*
  init_stepper()
  ----------------
  Configure I/O pins for the steppers and LEDs, initialize AccelStepper
  limits and register steppers with MultiStepper. Enables the stepper
  driver after configuration.
  Side-effects: sets pin modes, toggles LEDs, writes to `stepperEnable`.
*/
void init_stepper(){
  pinMode(rtStepPin, OUTPUT);
  pinMode(rtDirPin, OUTPUT);
  pinMode(ltStepPin, OUTPUT);
  pinMode(ltDirPin, OUTPUT);
  pinMode(stepperEnable, OUTPUT);
  digitalWrite(stepperEnable, stepperEnFalse); // keep drivers disabled until configured

  pinMode(enableLED, OUTPUT);
  digitalWrite(enableLED, LOW);

  // LEDs used to show state during startup
  pinMode(redLED, OUTPUT);
  pinMode(grnLED, OUTPUT);
  pinMode(ylwLED, OUTPUT);
  digitalWrite(redLED, HIGH);
  digitalWrite(ylwLED, HIGH);
  digitalWrite(grnLED, HIGH);
  delay(pauseTime / 5);
  digitalWrite(redLED, LOW);
  digitalWrite(ylwLED, LOW);
  digitalWrite(grnLED, LOW);

  // Configure stepper performance limits and register them with MultiStepper
  stepperRight.setMaxSpeed(max_speed);
  stepperRight.setAcceleration(max_accel);
  stepperLeft.setMaxSpeed(max_speed);
  stepperLeft.setAcceleration(max_accel);
  steppers.addStepper(stepperRight);
  steppers.addStepper(stepperLeft);

  // Enable the stepper drivers after configuration
  digitalWrite(stepperEnable, stepperEnTrue);
  digitalWrite(enableLED, HIGH);
}

/*
  randomWander()
  ---------------
  Periodically sets both wheel speeds to random values to create
  a wandering motion. Uses `reorientInterval` to throttle updates.
*/
void randomWander(){
  if (millis() - lastReorientRequest >= reorientInterval) {
    lastReorientRequest = millis();
    stepperLeft.setSpeed(random(200, 500));
    stepperRight.setSpeed(random(200, 500));
  }
}

/*
  goToAngle(int angle)
  ---------------------
  Rotate the robot approximately `angle` degrees by commanding
  opposite wheel travel via MultiStepper. Blocking call until move
  completes.
  Inputs: `angle` (degrees)
*/
void goToAngle(int angle){
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  int numSteps = angle*WIDTH_OF_BOT_CM/2*CM_TO_STEPS_CONV;

  long positions[2]; // Array of desired stepper positions
  positions[1] = -numSteps; // left motor target
  positions[0] = numSteps;  // right motor target
  steppers.moveTo(positions);

  int spinSpeedLocal = spinSpeed;
  if(numSteps < 0) spinSpeedLocal = -spinSpeedLocal;

  stepperLeft.setSpeed(-spinSpeedLocal); // left spin
  stepperRight.setSpeed(spinSpeedLocal); // right spin

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

/*
  collideDetection(SensorPacket p)
  --------------------------------
  Check sensor readings for obstacles within `maximumStoppingDist`.
  Returns true if any front/side sensor indicates a collision and
  updates state LEDs to show which sensor tripped.
  Inputs: `p` - sensor packet snapshot
  Returns: true if collision detected
*/
bool collideDetection(SensorPacket p){
  if(p.frontLidar < maximumStoppingDist && p.frontLidar > 0){
    // obstacle is too close in front
    digitalWrite(redLED, HIGH);
    return true;
  } else if(p.rightSonar < maximumStoppingDist && p.rightSonar > 0){
    digitalWrite(ylwLED, HIGH);
    return true;
  } else if(p.leftSonar < maximumStoppingDist && p.leftSonar > 0){
    digitalWrite(grnLED, HIGH);
    return true;
  }

  // clear state LEDs when no collision
  digitalWrite(grnLED, LOW);
  digitalWrite(redLED, LOW);
  digitalWrite(ylwLED, LOW);
  return false;
}

/*
  computeRepulsiveVector(float &Fx, float &Fy, float &angle)
  -----------------------------------------------------------
  Combine repulsive forces from all sensors into an (Fx,Fy) vector
  expressed in the robot frame and compute its heading in `angle`.
  Outputs: Fx, Fy (by reference), angle (radians, by reference)
*/
void computeRepulsiveVector(float &Fx, float &Fy, float &angle) {
  Fx = 0;
  Fy = 0;
  // Get forces from each sonar
  float fLeftSonar = repulsiveForce(sensorData.leftSonar);
  float fRightSonar = repulsiveForce(sensorData.rightSonar);
  // get forces from each lidar
  float fFrontLidar = repulsiveForce(sensorData.frontLidar);
  float fBackLidar = repulsiveForce(sensorData.backLidar);
  float fLeftLidar = repulsiveForce(sensorData.leftLidar);
  float fRightLidar = repulsiveForce(sensorData.rightLidar);
  // SONAR contributions (diagonal sensors)
  Fx -= fLeftSonar * cos(radians(ANGLE_LEFT_SONAR));
  Fy -= fLeftSonar * sin(radians(ANGLE_LEFT_SONAR));

  Fx -= fRightSonar * cos(radians(ANGLE_RIGHT_SONAR));
  Fy -= fRightSonar * sin(radians(ANGLE_RIGHT_SONAR));

  // LIDAR contributions (cardinal directions)
  Fx -= fFrontLidar * cos(radians(ANGLE_FRONT_LIDAR));  // Front pushes back
  Fy -= fFrontLidar * sin(radians(ANGLE_FRONT_LIDAR));

  Fx -= fBackLidar * cos(radians(ANGLE_BACK_LIDAR));    // Back pushes forward
  Fy -= fBackLidar * sin(radians(ANGLE_BACK_LIDAR));

  Fx -= fLeftLidar * cos(radians(ANGLE_LEFT_LIDAR));    // Left pushes right
  Fy -= fLeftLidar * sin(radians(ANGLE_LEFT_LIDAR));

  Fx -= fRightLidar * cos(radians(ANGLE_RIGHT_LIDAR));  // Right pushes left
  Fy -= fRightLidar * sin(radians(ANGLE_RIGHT_LIDAR));

  angle = atan2(Fy, Fx);
}

/*
  repulsiveForce(int distance)
  ----------------------------
  Compute a scalar repulsive force from a measured distance using an
  inverse-square-like law. Returns 0 for invalid/out-of-range
  distances.
  Inputs: `distance` (cm)
  Returns: clamped force magnitude
*/
float repulsiveForce(int distance) {
  if (distance <= 0 || distance > maximumStoppingDist) return 0;
  // Inverse square law with small stabilizer
  float force = K_REPULSIVE / (distance * distance + 1);
  return constrain(force, 0, MAX_FORCE);
}

/*
  runAwayBehavior()
  ------------------
  Escape behavior triggered when approached: if trapped, stop;
  otherwise compute an escape rotation using the repulsive field.
*/
void runAwayBehavior(){
  stepperLeft.setSpeed(0);
  stepperRight.setSpeed(0);
  // detect if there are obstacles on all sides
  if(sensorData.frontLidar > 0 && sensorData.leftLidar > 0 && sensorData.rightLidar > 0 && sensorData.backLidar > 0){
    // if obstacles on all sides and within trap threshold then stop
    if(sensorData.frontLidar < maximumStoppingDist+trapDetectionModifier && sensorData.leftLidar < maximumStoppingDist+trapDetectionModifier && sensorData.rightLidar < maximumStoppingDist+trapDetectionModifier && sensorData.backLidar < maximumStoppingDist+trapDetectionModifier){
      stepperRight.setSpeed(0);
      stepperLeft.setSpeed(0);
      return;
    }
  }

  // otherwise, perform an escape rotation based on computed repulsive angle
  goToAngle(forceVectorAngle);
}

/*
  followBehavior(float followAngle)
  -------------------------------
  Proportional follower controller: computes a base forward speed
  from distance error and a steering correction from side-sonar
  difference. Sets wheel speeds accordingly. `followAngle` is
  currently unused but reserved for heading-based control.
*/
void followBehavior(float followAngle){
  // Determine the closest measured distance among front/side sensors
  int minDistance = sensorData.leftSonar;
  if (sensorData.rightSonar < minDistance) minDistance = sensorData.rightSonar;
  if (sensorData.frontLidar < minDistance) minDistance = sensorData.frontLidar;

  // Calculate distance error (negative = too close, positive = too far)
  float distanceError = minDistance - targetFollowDist;

  float baseSpeed = KP_DISTANCE * distanceError;
  baseSpeed = constrain(baseSpeed, 0, maxSpeed);

  // If too close then stop
  if(distanceError < 0){
    stepperLeft.setSpeed(0);
    stepperRight.setSpeed(0);
    return;
  }

  // Steering error: difference between right and left sonar (pos => object to right)
  float steeringError = sensorData.rightSonar - sensorData.leftSonar;

  // Proportional steering correction
  float steeringCorrection = KP_STEERING * steeringError;
  steeringCorrection = constrain(steeringCorrection, -spinSpeed, spinSpeed);

  // Differential wheel speeds for steering
  float leftSpeed = baseSpeed - steeringCorrection;
  float rightSpeed = baseSpeed + steeringCorrection;

  stepperRight.setSpeed(rightSpeed);
  stepperLeft.setSpeed(leftSpeed);
}

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
      Serial.print(" | LSonar: "); Serial.print(sensorData.leftSonar);
      Serial.print(" | RSonar: "); Serial.print(sensorData.rightSonar);
      Serial.println();
    }
  }
}


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
}

void loop() {
  stepperLeft.runSpeed();
  stepperRight.runSpeed();
  updateSensorData();
  computeRepulsiveVector(forceVectorx, forceVectory, forceVectorAngle);
  



  if (printForceCalc){

    Serial.print("Fx = "); Serial.print(forceVectorx);
    Serial.print(" Fy = "); Serial.print(forceVectory);
    Serial.print(" Angle = "); Serial.println(forceVectorAngle*180/PI);
    
  }

  if (currentMode == COLLIDE_MODE) {
    if(collideDetection(sensorData)){
      stepperLeft.setSpeed(0);
      stepperRight.setSpeed(0);
    } else{
      stepperLeft.setSpeed(maxSpeed);
      stepperRight.setSpeed(maxSpeed);
    }
  } else if (currentMode == RUNAWAY_MODE) {
    if(!collideDetection(sensorData)){
      stepperLeft.setSpeed(maxSpeed);
      stepperRight.setSpeed(maxSpeed);
    } else{
      stepperLeft.setSpeed(0);
      stepperRight.setSpeed(0);
      runAwayBehavior(); // RUNAWAY: Sits still, runs away using potential fields when approached
    }
  } else if (currentMode == FOLLOW_MODE) {
      followBehavior(forceVectorAngle); // FOLLOW: Curious kid follows object at target distance with proportional control
  } else if (currentMode == RANDOM_WANDER) {
      randomWander(); // RANDOM WANDER: Wanders randomly
  } else if (currentMode == SMART_WANDER_MODE){
      // SMART WANDER: State machine (wander -> collide -> avoid)
      if(collideDetection(sensorData)){
        runAwayBehavior();
      } else{
        randomWander();
      }
  } else if (currentMode == SMART_FOLLOW_MODE){
      if(!collideDetection(sensorData)){
        randomWander();
      } else{
        followBehavior(forceVectorAngle);
      }
  }

  

  stepperLeft.runSpeed();
  stepperRight.runSpeed();
}
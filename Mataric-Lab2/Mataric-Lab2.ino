//includew all necessary libraries
#include <Arduino.h>//include for PlatformIO Ide
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library

//state LEDs connections
#define redLED 5            //red LED for displaying states
#define greenLED 6            //green LED for displaying states
#define grnLED 6            //green LED for displaying states
#define yellowLED 7            //yellow LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED
int leds[3] = {5,6,7};      //array of LED pin numbers

//define motor pin numbers
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

int pauseTime = 2500;   //time before robot moves
int stepTime = 500;     //delay time between high and low on step pin
int wait_time = 1000;   //delay for printing data

int minIntervalAngle = 13.79; // degrees 
int leftSpd = 1000; // default speed for left and right motors in septs per second
int rightSpd = 1000;

int leftSonarDist = 1000; // default left sonar distance
int rightSonarDist = 1000; // default right sonar distance

//define encoder pins
#define LEFT 0        //left encoder
#define RIGHT 1       //right encoder
const int ltEncoder = 18;        //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;        //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset

// Analog sonar
#define leftSnrPin 4
#define rightSnrPin 3
volatile int sonarDistance = 1000; // previous sonar distance, default is 1000 cm
volatile bool obstacleDetected = false;

// Lidar sensors
#define frontLdr 8
#define backLdr 9
#define leftLdr 10
#define rightLdr 12
int frontLidarDist; // front lidar distance
int backLidarDist; // front lidar distance
int leftLidarDist; // front lidar distance
int rightLidarDist; // front lidar distance

// Behavior constants
const int OBSTACLE_THRESHOLD = 10;   // cm - detection threshold
const int SAFE_DISTANCE = 40;        // cm - desired safe distance
const float K_REPULSIVE = 800.0;     // Repulsive force constant
const int MAX_FORCE = 100;           // Maximum force magnitude
const int MAX_SPEED = 800;           // Maximum speed for runaway

// Sensor angles relative to robot (degrees)
const float ANGLE_LEFT_SONAR = 45.0;     // Left sonar at 45 degrees
const float ANGLE_RIGHT_SONAR = -45.0;   // Right sonar at -45 degrees
const float ANGLE_FRONT_LIDAR = 0.0;     // Front LIDAR at 0 degrees
const float ANGLE_BACK_LIDAR = 180.0;    // Back LIDAR at 180 degrees
const float ANGLE_LEFT_LIDAR = 90.0;     // Left LIDAR at 90 degrees
const float ANGLE_RIGHT_LIDAR = -90.0;   // Right LIDAR at -90 degrees

// Local minima detection
unsigned long lastMoveTime = 0;
int lastAngle = 0;
int oscillationCount = 0;
const int OSCILLATION_THRESHOLD = 4;

// Behavior modes
enum BehaviorMode {
  COLLIDE_MODE,    // Aggressive kid: drives forward, stops at obstacle
  RUNAWAY_MODE,    // Shy kid: sits still, runs away when approached
  FOLLOW_MODE,     // Curious kid: follows object at target distance
  RANDOM_WANDER    // Random wander behavior
};
BehaviorMode currentMode = FOLLOW_MODE;  // Change this to switch behaviors

// State machine for non-blocking runaway behavior
enum RunawayState {
  RUNAWAY_IDLE,      // Sitting still
  RUNAWAY_TURNING,   // Turning away
  RUNAWAY_MOVING     // Moving forward
  SMART_WANDER_MODE  // Smart wander with state machine (wander -> collide -> avoid)
};
RunawayState runawayState = RUNAWAY_IDLE;

// Follow behavior constants
const int TARGET_FOLLOW_DISTANCE = 10;  // cm - desired distance from object
const float KP_DISTANCE = 50.0;          // Proportional gain for distance control
const float KP_STEERING = 3.0;           // Proportional gain for steering
const int MIN_FOLLOW_SPEED = 100;        // Minimum motor speed
const int MAX_FOLLOW_SPEED = 800;        // Maximum motor speed
const int FOLLOW_DETECT_THRESHOLD = 50;  // cm - max distance to detect object to follow

// STATE MACHINE for Smart Wander Mode
enum SmartWanderState {
  SW_WANDERING,     // GREEN LED - randomly wander
  SW_COLLIDE,       // RED LED - obstacle detected, stopped
  SW_AVOIDING       // YELLOW LED - avoiding obstacle
};

SmartWanderState smartWanderState = SW_WANDERING;

// Smart Wander constants
const int DANGER_DISTANCE = 15;      // cm - danger threshold for obstacle detection
const int CLEAR_DISTANCE = 35;       // cm - distance to be considered "clear" of obstacle

// Smart Wander variables
unsigned long lastWanderTime = 0;
const unsigned long WANDER_INTERVAL = 3000; // Change direction every 3 seconds
bool wanderMoving = false;

// Avoid behavior variables
enum AvoidState {
  AVOID_TURNING,
  AVOID_MOVING,
  AVOID_COMPLETE
};
AvoidState avoidState = AVOID_TURNING;

// Constant Vars
#define INCHES_TO_CM 2.54 //conversion factor from inches to centimeters
#define TWO_FEET_IN_STEPS 1848 //number of steps to move robot forward 2 feet
#define WIDTH_OF_BOT_CM 21.2 //width of robot in centimeters, distance between wheels center to center
#define WHEEL_DIAM_CM 8.4 // wheel diameter in centimeters
#define STEPS_PER_ROT 800 //number of steps per wheel rotation
#define CM_PER_ROTATION 26.39 //circumference of wheel in centimeters
#define CM_TO_STEPS_CONV STEPS_PER_ROT/CM_PER_ROTATION //conversion factor from centimeters to steps
#define ENCODER_TICKS_PER_ROTATION 20 //number of encoder ticks per wheel rotation
#define CM_PER_FOOT 30.48 // number of centimeters in a foot

#define DEFAULT_SPEED 500 // default motor speed

//interrupt function to count left encoder tickes
void LwheelSpeed() {
  encoder[LEFT] ++;  //count the left wheel encoder interrupts
}
//interrupt function to count right encoder ticks
void RwheelSpeed() {
  encoder[RIGHT] ++; //count the right wheel encoder interrupts
}
//function to set all stepper motor variables, outputs and LEDs
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
//function prints encoder data to serial monitor
void print_encoder_data() {
  static unsigned long timer = 0;                           //print manager timer
  if (millis() - timer > 100) {                             //print encoder data every 100 ms or so
    lastSpeed[LEFT] = encoder[LEFT];                        //record the latest left speed value
    lastSpeed[RIGHT] = encoder[RIGHT];                      //record the latest right speed value
    accumTicks[LEFT] = accumTicks[LEFT] + encoder[LEFT];    //record accumulated left ticks
    accumTicks[RIGHT] = accumTicks[RIGHT] + encoder[RIGHT]; //record accumulated right ticks
    Serial.println("Encoder value:");
    Serial.print("\tLeft:\t");
    Serial.print(encoder[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(encoder[RIGHT]);
    Serial.println("Accumulated Ticks: ");
    Serial.print("\tLeft:\t");
    Serial.print(accumTicks[LEFT]);
    Serial.print("\tRight:\t");
    Serial.println(accumTicks[RIGHT]);
    encoder[LEFT] = 0;                          //clear the left encoder data buffer
    encoder[RIGHT] = 0;                         //clear the right encoder data buffer
    timer = millis();                           //record current time since program started
  }
}
/*
  Turns off all the LEDs
*/
void turnOffLEDs(){
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, LOW);
  digitalWrite(yellowLED, LOW);
}
// ----------------- MOTOR FUNCTIONS -----------------------------------
/*
  Stops both left and right motors.
*/
void stop(){
  stepperRight.stop();
  stepperLeft.stop();
}
/*function to run both wheels to a position at speed*/
void runAtSpeedToPosition() {
  stepperRight.runSpeedToPosition();
  stepperLeft.runSpeedToPosition();
}
/*function to run both wheels continuously at a speed*/
void runAtSpeed ( void ) {
  while (stepperRight.runSpeed() || stepperLeft.runSpeed()) {
  }
}
/*
  Turn the robot to a specified angle, rotating around its center
  Input angle is in degrees.
*/
void goToAngle(int angle){
  turnOffLEDs();
  digitalWrite(greenLED, HIGH);
  encoder[LEFT] = 0;
  encoder[RIGHT] = 0;
  
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

  Serial.println("Go To Angle: " + String(angle));
  
  double angle_rad = angle*PI/180;
  
  double distanceToTravel = angle_rad*WIDTH_OF_BOT_CM/2;
  
  int numSteps = angle_rad*WIDTH_OF_BOT_CM/2*CM_TO_STEPS_CONV;
  Serial.println(String("numSteps: ")+ numSteps);
  
  long positions[2]; // Array of desired stepper positions
  positions[1] = -numSteps; //left motor position
  positions[0] = numSteps; //right motor position
  steppers.moveTo(positions);

  int spinSpeed = 200;
  if(numSteps < 0){
    spinSpeed = -spinSpeed;
  }
  stepperLeft.setSpeed(-spinSpeed);//set left motor speed
  stepperRight.setSpeed(spinSpeed);//set right motor speed

  steppers.runSpeedToPosition(); // Blocks until all are in position
  turnOffLEDs();
}
/*
  Turn robot to an angle, go forward to a particular direction.
  Input x value is in centimeters.
  Input y value is in centimeters.

  Accounts for odometry error. (unimplemented)
*/
void goToGoal(int x, int y){
  encoder[LEFT] = 0;
  encoder[RIGHT] = 0;

  turnOffLEDs();
  digitalWrite(greenLED, HIGH); digitalWrite(yellowLED, HIGH);
  
  Serial.println("Go To Goal");
  double angle_rad = atan2(y,x);
  int angle = angle_rad * (180/PI);
  goToAngle(angle);
  digitalWrite(greenLED, HIGH); digitalWrite(yellowLED, HIGH);

  double forwardDistance_cm =  sqrt(sq(x)+sq(y)); // go forward by the hypotenuse
  double forwardDistance_ft = forwardDistance_cm / CM_PER_FOOT;
  Serial.println(String("  going forward by ") + forwardDistance_ft + (" ft"));
  forward(forwardDistance_cm);
  
  Serial.println("Starting goToGoal Error Correction...");
  delay(1000);
  // goToGoalErrorCorrection(forwardDistance_cm);

  Serial.println("Turning off led's for goal");
  turnOffLEDs();
}

/*
  Moves the robot forward by a specified distance
  input distance is in centimeters
*/
void forward(int distance) {
  Serial.println("Forward");
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  int distance_step = distance*CM_TO_STEPS_CONV;
  stepperLeft.moveTo(distance_step);//left motor absolute position
  stepperRight.moveTo(distance_step);//right motor absolute position
  stepperLeft.setSpeed(leftSpd);
  stepperRight.setSpeed(rightSpd);

  steppers.runSpeedToPosition();
}

/*
  This function generates a random number.
  This function is called by the randomWander function.

  This input is maxVal, the maximum value of the randomly generated number.
  This outputs a random number.
*/
int getRandomNumber(int minVal, int maxVal){
  randomSeed(analogRead(A0));
  return random(minVal, maxVal);
}
/*
  reads a lidar given a pin
  Returns a distance value in cm
*/
int read_lidar(int pin) {
  int d;
  int16_t t = pulseIn(pin, HIGH, 25000); // 25 ms timeout to prevent blocking
  d = (t - 1000) * 3 / 40;
  if (t == 0 || t > 1850 || d < 0) { d = 0; }
  return d;
}
// reads a sonar given a pin
int read_sonar(int pin) {
  float velocity((331.5 + 0.6 * (float)(20)) * 100 / 1000000.0);
  uint16_t distance, pulseWidthUs;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  digitalWrite(pin, HIGH);            //Set the trig pin High
  delayMicroseconds(10);              //Delay of 10 microseconds
  digitalWrite(pin, LOW);             //Set the trig pin Low
  pinMode(pin, INPUT);                //Set the pin to input mode
  pulseWidthUs = pulseIn(pin, HIGH);  //Detect the high level time on the echo pin, the output high level time represents the ultrasonic flight time (unit: us)
  distance = pulseWidthUs * velocity / 2.0;
  if (distance < 0 || distance > 50) { distance = 0; }
  return distance;
}

/*
  Update both left and right sonars.
*/
void updateSonarReadings() {
  static unsigned long lastSampleTime = 0;
  const unsigned long SAMPLE_PERIOD = 50; // ms (20 Hz)

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_PERIOD) {
    lastSampleTime = now;
    leftSonarDist = read_sonar(leftSnrPin);
    rightSonarDist = read_sonar(rightSnrPin);
  }
}
/*
  Updates the distance values of each of the lidar sensors.
  reads all 4 sensors at intervals for non-blocking.
*/
void updateLidarReadings() {
  // static makes it initialize only once, retains previous value on next iterations
  static unsigned long lastSampleTime = 0;
  static int sensorIndex = 0; // Which sensor to read next
  const unsigned long SAMPLE_PERIOD = 10; // Read one sensor every 100ms

  unsigned long now = millis();
  if (now - lastSampleTime >= SAMPLE_PERIOD) {
    lastSampleTime = now;
    
    // Read one sensor at a time to minimize blocking
    switch(sensorIndex) {
      case 0:
        frontLidarDist = read_lidar(frontLdr);
        Serial.print("F:");
        Serial.print(frontLidarDist);
        break;
      case 1:
        backLidarDist = read_lidar(backLdr);
        Serial.print(" B:");
        Serial.print(backLidarDist);
        break;
      case 2:
        leftLidarDist = read_lidar(leftLdr);
        Serial.print(" L:");
        Serial.print(leftLidarDist);
        break;
      case 3:
        rightLidarDist = read_lidar(rightLdr);
        Serial.print(" R:");
        Serial.println(rightLidarDist);
        break;
    }
    
    // Move to next sensor (cycle through 0-3)
    sensorIndex = (sensorIndex + 1) % 4;
  }
}
/*
  Makes the robot wander randomly.
  The maximum distance it'll wander in one motion is 30 cm.
  The maximum angle the robot will rotate is by 180 deg to the left or right.
  
  There are no inputs.
*/
// ------------------ random wanderer behavior -----------------------------
void randomWander(){
  turnOffLEDs();
  digitalWrite(greenLED, HIGH);
  int randomAngle = getRandomNumber(15, 360); // min of 15 deg, max of 360 deg
  int randomDistance = getRandomNumber(15, 50); // min of 15 cm, maximum of 50 cm

  if (abs(randomAngle) < 15) {
    randomAngle = (randomAngle < 0) ? -15 : 15;
  }
  goToAngle(randomAngle);
  forward(randomDistance);
  
}
// --------------------- collide behavior ----------------------------------
/*
  Robot drives until it encounters an obstacle.
  Robot stops when an obstacle is detected and continues moving when the object is removed.
  Robot should stop without hitting the object.
  Turns on the red LED when executing this behavior.
*/
void collideBehavior() {
  // Check if obstacle is detected on either sensor
  int sensorType = 0;
  // if(leftSonarDist < OBSTACLE_THRESHOLD){
  //   obstacleDetected = true;
  //   sensorType = 1;
  // }
  // else if(rightSonarDist < OBSTACLE_THRESHOLD){
  //   obstacleDetected = true;
  //   sensorType = 2;
  // }
  if(frontLidarDist < OBSTACLE_THRESHOLD && frontLidarDist > 0){
    obstacleDetected = true;
    sensorType = 3;
  }
  else{
    obstacleDetected = false;
  }
  // obstacleDetected = (leftSonarDist < OBSTACLE_THRESHOLD) || (rightSonarDist < OBSTACLE_THRESHOLD) || (frontLidarDistance < OBSTACLE_THRESHOLD);
  
  if (obstacleDetected) {
    stop(); // STOP immediately when obstacle detected
    digitalWrite(redLED, HIGH);
    digitalWrite(yellowLED, LOW);
    digitalWrite(greenLED, LOW);
    
    Serial.print("COLLIDE: Obstacle! ");
    if(sensorType == 1){
      Serial.println("left sonar: " + String(leftSonarDist) + " cm");
    }
    else if(sensorType == 2){
      Serial.println("right sonar: " + String(rightSonarDist) + " cm");
    }
    else if(sensorType == 3){
      Serial.println("front lidar: " + String(frontLidarDist) + " cm");
    }
  } else {
    // No obstacle - drive forward continuously
    digitalWrite(redLED, HIGH);
    digitalWrite(yellowLED, LOW);
    digitalWrite(greenLED, LOW);
    
    // Set continuous forward motion
    stepperLeft.setSpeed(leftSpd);
    stepperRight.setSpeed(rightSpd);
    
    // Run motors non-blocking
    stepperLeft.runSpeed();
    stepperRight.runSpeed();
  }
}

// ----------------- Run Away behavior -------------------------------------
// Calculate repulsive force from a single sensor
float repulsiveForce(int distance) {
  if (distance <= 0 || distance > OBSTACLE_THRESHOLD) return 0;
  // Inverse square law with safe distance
  float force = K_REPULSIVE / (distance * distance + 1);
  return constrain(force, 0, MAX_FORCE);
}

/*
  Compute the repulsive vector from both sensors.
  Passes input by reference, changing Fx, Fy will change the original.
  Reference is used because we want to return two values from one function.
*/
void computeRepulsiveVector(float &Fx, float &Fy) {
  Fx = 0;
  Fy = 0;
  // Get forces from each sonar
  float fLeftSonar = repulsiveForce(leftSonarDist);
  float fRightSonar = repulsiveForce(rightSonarDist);
  // get forces from each lidar
  float fFrontLidar = repulsiveForce(frontLidarDist);
  float fBackLidar = repulsiveForce(backLidarDist);
  float fLeftLidar = repulsiveForce(leftLidarDist);
  float fRightLidar = repulsiveForce(rightLidarDist);
  
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
}
// Detect if robot is oscillating (stuck in local minima)
bool detectOscillation(int currentAngle) {
  if (abs(currentAngle - lastAngle) > 150) {
    oscillationCount++;
  } else {
    oscillationCount = 0;
  }
  
  lastAngle = currentAngle;
  return (oscillationCount >= OSCILLATION_THRESHOLD);
}
// NON-BLOCKING turn function
void turnNonBlocking(int angle) {
  if (abs(angle) < 5) return; // ignores small angles
  
  double angle_rad = angle * PI / 180.0;
  int numSteps = angle_rad * WIDTH_OF_BOT_CM / 2.0 * CM_TO_STEPS_CONV;
  
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  stepperLeft.moveTo(-numSteps);
  stepperRight.moveTo(numSteps);
  
  int spinSpeed = 500;
  if(numSteps < 0) spinSpeed = -spinSpeed;
  
  stepperLeft.setSpeed(-spinSpeed);
  stepperRight.setSpeed(spinSpeed);
}

// NON-BLOCKING forward function
void forwardNonBlocking(int distance) {
  int distance_step = distance * CM_TO_STEPS_CONV;
  
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  stepperLeft.moveTo(distance_step);
  stepperRight.moveTo(distance_step);
  stepperLeft.setSpeed(leftSpd);
  stepperRight.setSpeed(rightSpd);
}

void runawayBehavior() {
  // calculate repulsive force field from all sensors
  float Fx, Fy;
  computeRepulsiveVector(Fx, Fy);
  float magnitude = sqrt(Fx*Fx + Fy*Fy);
  
  // check for any obstacles nearby (any sensor)
  bool obstacleNearby = (leftSonarDist > 0 && leftSonarDist < OBSTACLE_THRESHOLD) || 
                        (rightSonarDist > 0 && rightSonarDist < OBSTACLE_THRESHOLD) ||
                        (frontLidarDist > 0 && frontLidarDist < OBSTACLE_THRESHOLD) ||
                        (backLidarDist > 0 && backLidarDist < OBSTACLE_THRESHOLD) ||
                        (leftLidarDist > 0 && leftLidarDist < OBSTACLE_THRESHOLD) ||
                        (rightLidarDist > 0 && rightLidarDist < OBSTACLE_THRESHOLD);
  
  switch(runawayState) {
    case RUNAWAY_IDLE: // robot sits still
      if (magnitude > 0.1 && obstacleNearby) { // Obstacle detected - start turning away
        digitalWrite(yellowLED, HIGH);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, LOW);
        // calculate escape angle from force vectors
        float turnAngle = atan2(Fy, Fx) * 180.0 / PI;
        
        if (detectOscillation(turnAngle)) { // if oscillating
          // Escape local minima with random large turn
          digitalWrite(redLED, HIGH);
          randomSeed(analogRead(A0));
          int escapeAngle = random(90, 180);
          if (random(0, 2) == 0) escapeAngle = -escapeAngle;
          turnNonBlocking(escapeAngle);
          oscillationCount = 0;
        } else {
          // normal escape - turn towards calculated escape angle
          turnNonBlocking(turnAngle);
        }
        
        Serial.print("RUNAWAY: Turning away, angle: ");
        Serial.println(turnAngle);
        
        runawayState = RUNAWAY_TURNING;
      } else {
        // No obstacle - sit still
        digitalWrite(yellowLED, LOW);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, LOW);
        stop();
        oscillationCount = 0;
        
        // print runaway status
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 1000) {
          Serial.println("RUNAWAY: Idle, waiting...");
          lastPrint = millis();
        }
      }
      break;
      
    case RUNAWAY_TURNING: // robot is turning away from obstacles
      // Non-blocking turn
      if (stepperLeft.runSpeed() | stepperRight.runSpeed()) {
        // Still turning
      } else {
        // Turn complete - start moving forward
          // convert force magnitude (0-100) to distance (10-40 cm)
        int escapeDistance = map(magnitude, 0, MAX_FORCE, 10, 40); // cm
        escapeDistance = constrain(escapeDistance, 10, 40);
        forwardNonBlocking(escapeDistance);
        
        Serial.print("RUNAWAY: Moving forward ");
        Serial.print(escapeDistance);
        Serial.println("cm");
        
        runawayState = RUNAWAY_MOVING;
      }
      break;
      
    case RUNAWAY_MOVING:
      // Non-blocking forward movement
      if (stepperLeft.runSpeed() | stepperRight.runSpeed()) {
        // Still moving - check for NEW obstacles appearing
        float newFx, newFy;
        computeRepulsiveVector(newFx, newFy);
        float newMagnitude = sqrt(newFx*newFx + newFy*newFy);
        
        if (newMagnitude > magnitude * 1.5) {
          // NEW threat detected while moving! Return to idle to reassess
          Serial.println("RUNAWAY: New threat detected during move!");
          runawayState = RUNAWAY_IDLE;
        }
      } else {
        // Movement complete - return to idle state
        Serial.println("RUNAWAY: Move complete, back to idle");
        runawayState = RUNAWAY_IDLE;
      }
      break;
  }
}

// ============================= Follow behavior ============================================
// FOLLOW BEHAVIOR (Curious Kid) - NON-BLOCKING with Proportional Control
// Robot follows object at target distance using proportional control
// Speeds up when object moves away, slows down when object approaches
// Uses steering to keep object centered
// YELLOW + GREEN LEDs indicate active following
void followBehavior() {  
  // Get minimum distance (closest object)
  int minDistance = leftSonarDist;
  if (rightSonarDist < minDistance) minDistance = rightSonarDist;
  if (frontLidarDist < minDistance) minDistance = frontLidarDist;
  
  // Check if object is detected within range
  bool objectDetected = (minDistance < FOLLOW_DETECT_THRESHOLD);
  
  if (!objectDetected) {
    // No object detected - HALT (collide behavior)
    stop();
    digitalWrite(redLED, HIGH);
    digitalWrite(yellowLED, LOW);
    digitalWrite(greenLED, LOW);
    
    Serial.println("FOLLOW: No object detected - HALTED");
    return;
  }
  // Object detected - FOLLOW with proportional control
  digitalWrite(redLED, LOW);
  digitalWrite(yellowLED, HIGH);  // Curious kid active
  digitalWrite(greenLED, HIGH);   // Curious kid active
  
  // Calculate distance error (negative = too close, positive = too far)
  float distanceError = minDistance - TARGET_FOLLOW_DISTANCE;
  
  // Calculate steering error (difference between left and right sensors)
  // Negative = object on left, Positive = object on right
  float steeringError = rightSonarDist - leftSonarDist;
  
  // Proportional control for forward/backward speed
  float baseSpeed = KP_DISTANCE * distanceError;
  baseSpeed = constrain(baseSpeed, 0, MAX_FOLLOW_SPEED);
  
  // Proportional control for steering correction
  float steeringCorrection = KP_STEERING * steeringError;
  steeringCorrection = constrain(steeringCorrection, -200, 200);
  
  // Calculate differential motor speeds for steering while moving
  float leftSpeed = baseSpeed - steeringCorrection;
  float rightSpeed = baseSpeed + steeringCorrection;
  
  // Apply minimum speed threshold (deadband for small errors)
  if (abs(distanceError) < 3) { // Within acceptable range - stop
    leftSpeed = 0;
    rightSpeed = 0;
  } else {
    // Ensure minimum speed when moving
    if (leftSpeed > 0 && leftSpeed < MIN_FOLLOW_SPEED) leftSpeed = MIN_FOLLOW_SPEED;
    if (leftSpeed < 0 && leftSpeed > -MIN_FOLLOW_SPEED) leftSpeed = -MIN_FOLLOW_SPEED;
    if (rightSpeed > 0 && rightSpeed < MIN_FOLLOW_SPEED) rightSpeed = MIN_FOLLOW_SPEED;
    if (rightSpeed < 0 && rightSpeed > -MIN_FOLLOW_SPEED) rightSpeed = -MIN_FOLLOW_SPEED;
  }
  
  // Constrain speeds to motor limits
  leftSpeed = constrain(leftSpeed, 0, MAX_FOLLOW_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_FOLLOW_SPEED);
  
  // Apply speeds to motors (non-blocking)
  stepperLeft.setSpeed(leftSpeed);
  stepperRight.setSpeed(rightSpeed);
  stepperLeft.runSpeed();
  stepperRight.runSpeed();
  
  // Debug output
  Serial.print("FOLLOW: Dist=");
  Serial.print(minDistance);
  Serial.print("cm Error=");
  Serial.print(distanceError);
  Serial.print("cm SteerErr=");
  Serial.print(steeringError);
  Serial.print(" | Spd L=");
  Serial.print(leftSpeed);
  Serial.print(" R=");
  Serial.println(rightSpeed);
}

// ================================== Smart Wander Behavior ==========================================
// Helper functions
/*
  Check if any sensor detects an obstacle within danger distance
*/
bool checkObstacleInDanger() {
  return (leftSonarDist > 0 && leftSonarDist < DANGER_DISTANCE) ||
         (rightSonarDist > 0 && rightSonarDist < DANGER_DISTANCE) ||
         (frontLidarDist > 0 && frontLidarDist < DANGER_DISTANCE) ||
         (leftLidarDist > 0 && leftLidarDist < DANGER_DISTANCE) ||
         (rightLidarDist > 0 && rightLidarDist < DANGER_DISTANCE);
}
/*
  Check if robot is clear of all obstacles
*/
bool checkClearOfObstacles() {
  return (leftSonarDist == 0 || leftSonarDist > CLEAR_DISTANCE) &&
         (rightSonarDist == 0 || rightSonarDist > CLEAR_DISTANCE) &&
         (frontLidarDist == 0 || frontLidarDist > CLEAR_DISTANCE) &&
         (leftLidarDist == 0 || leftLidarDist > CLEAR_DISTANCE) &&
         (rightLidarDist == 0 || rightLidarDist > CLEAR_DISTANCE);
}
/*
  Get the direction of the closest obstacle
  Returns angle in degrees
*/
float getObstacleDirection() {
  int minDist = 1000;
  float obstacleAngle = 0;
  
  // angle is equivalent to sensor closest to the object
  if (leftSonarDist > 0 && leftSonarDist < minDist) {
    minDist = leftSonarDist;
    obstacleAngle = ANGLE_LEFT_SONAR;
  }
  if (rightSonarDist > 0 && rightSonarDist < minDist) {
    minDist = rightSonarDist;
    obstacleAngle = ANGLE_RIGHT_SONAR;
  }
  if (frontLidarDist > 0 && frontLidarDist < minDist) {
    minDist = frontLidarDist;
    obstacleAngle = ANGLE_FRONT_LIDAR;
  }
  if (leftLidarDist > 0 && leftLidarDist < minDist) {
    minDist = leftLidarDist;
    obstacleAngle = ANGLE_LEFT_LIDAR;
  }
  if (rightLidarDist > 0 && rightLidarDist < minDist) {
    minDist = rightLidarDist;
    obstacleAngle = ANGLE_RIGHT_LIDAR;
  }
  return obstacleAngle;
}
/*
  Smart Wander with State Machine
  Combines wandering, collision detection, and avoidance
  GREEN LED = wandering, RED LED = collide, YELLOW LED = avoiding
*/
void smartWanderStateMachine() {
  
  switch(smartWanderState) {
    
    case SW_WANDERING:
      // GREEN LED on while wandering
      turnOffLEDs();
      digitalWrite(greenLED, HIGH);
      
      unsigned long currentTime = millis();
      
      // Check if it's time to start a new wander move
      if (!wanderMoving && (currentTime - lastWanderTime >= WANDER_INTERVAL)) {
        // Generate new random wander parameters
        int wanderAngle = getRandomNumber(-90, 90); // Turn -90 to 90 degrees
        int wanderDistance = getRandomNumber(20, 40); // Move 20-40 cm
        
        // Start the turn
        double angle_rad = wanderAngle * PI / 180.0;
        int numSteps = angle_rad * WIDTH_OF_BOT_CM / 2.0 * CM_TO_STEPS_CONV;
        
        stepperLeft.setCurrentPosition(0);
        stepperRight.setCurrentPosition(0);
        stepperLeft.moveTo(-numSteps);
        stepperRight.moveTo(numSteps);
        
        int spinSpeed = 300;
        if(numSteps < 0) spinSpeed = -spinSpeed;
        
        stepperLeft.setSpeed(-spinSpeed);
        stepperRight.setSpeed(spinSpeed);
        
        wanderMoving = true;
        Serial.print("SW_WANDER: Turn ");
        Serial.print(wanderAngle);
        Serial.print("deg, Move ");
        Serial.print(wanderDistance);
        Serial.println("cm");
      }
      
      // Continue executing the wander move
      if (wanderMoving) {
        // Check if turn is complete
        if (stepperLeft.distanceToGo() == 0 && stepperRight.distanceToGo() == 0) {
          // If we just finished turning, start moving forward
          if (stepperLeft.targetPosition() != 0) {
            // Start forward movement
            int wanderDistance = getRandomNumber(20, 40);
            int distance_step = wanderDistance * CM_TO_STEPS_CONV;
            stepperLeft.setCurrentPosition(0);
            stepperRight.setCurrentPosition(0);
            stepperLeft.moveTo(distance_step);
            stepperRight.moveTo(distance_step);
            stepperLeft.setSpeed(500);
            stepperRight.setSpeed(500);
          } else {
            // Movement complete
            wanderMoving = false;
            lastWanderTime = currentTime;
          }
        } else {
          // Continue moving
          stepperLeft.runSpeed();
          stepperRight.runSpeed();
        }
      }
      
      // Check for obstacle in danger distance
      if (checkObstacleInDanger()) {
        Serial.println("STATE TRANSITION: SW_WANDERING -> SW_COLLIDE");
        smartWanderState = SW_COLLIDE;
        stop(); // Stop immediately
        wanderMoving = false; // Reset wander flag
      }
      break;
      
    case SW_COLLIDE:
      // RED LED on when obstacle detected
      stop();
      turnOffLEDs();
      digitalWrite(redLED, HIGH);
      
      Serial.println("SW_COLLIDE: Obstacle detected in danger zone!");
      
      // Immediately transition to avoidance
      smartWanderState = SW_AVOIDING;
      avoidState = AVOID_TURNING; // Reset avoid state machine
      Serial.println("STATE TRANSITION: SW_COLLIDE -> SW_AVOIDING");
      break;
      
    case SW_AVOIDING:
      // YELLOW LED on while avoiding
      turnOffLEDs();
      digitalWrite(yellowLED, HIGH);
      
      switch(avoidState) {
        case AVOID_TURNING:
          // If this is the first time in this state, set up the turn
          if (stepperLeft.distanceToGo() == 0 && stepperRight.distanceToGo() == 0) {
            // Get obstacle direction and turn away from it
            float obstacleAngle = getObstacleDirection();
            int avoidTurnAngle;
            
            // Turn away from obstacle (opposite direction)
            if (obstacleAngle >= 0) {
              // Obstacle on right or front-right, turn left
              avoidTurnAngle = getRandomNumber(60, 120);
            } else {
              // Obstacle on left or front-left, turn right
              avoidTurnAngle = -getRandomNumber(60, 120);
            }
            
            // Set up turn
            double angle_rad = avoidTurnAngle * PI / 180.0;
            int numSteps = angle_rad * WIDTH_OF_BOT_CM / 2.0 * CM_TO_STEPS_CONV;
            
            stepperLeft.setCurrentPosition(0);
            stepperRight.setCurrentPosition(0);
            stepperLeft.moveTo(-numSteps);
            stepperRight.moveTo(numSteps);
            
            int spinSpeed = 400;
            if(numSteps < 0) spinSpeed = -spinSpeed;
            
            stepperLeft.setSpeed(-spinSpeed);
            stepperRight.setSpeed(spinSpeed);
            
            Serial.print("SW_AVOID: Turning ");
            Serial.print(avoidTurnAngle);
            Serial.println(" degrees away from obstacle");
          }
          
          // Execute turn
          if (stepperLeft.runSpeed() | stepperRight.runSpeed()) {
            // Still turning
          } else {
            // Turn complete, move to forward motion
            avoidState = AVOID_MOVING;
          }
          break;
          
        case AVOID_MOVING:
          // If this is the first time in this state, set up forward movement
          if (stepperLeft.distanceToGo() == 0 && stepperRight.distanceToGo() == 0) {
            int avoidDistance = getRandomNumber(30, 50); // Move 30-50 cm
            int distance_step = avoidDistance * CM_TO_STEPS_CONV;
            
            stepperLeft.setCurrentPosition(0);
            stepperRight.setCurrentPosition(0);
            stepperLeft.moveTo(distance_step);
            stepperRight.moveTo(distance_step);
            stepperLeft.setSpeed(600);
            stepperRight.setSpeed(600);
            
            Serial.print("SW_AVOID: Moving forward ");
            Serial.print(avoidDistance);
            Serial.println("cm");
          }
          
          // Execute forward movement
          if (stepperLeft.runSpeed() | stepperRight.runSpeed()) {
            // Still moving
          } else {
            // Movement complete
            avoidState = AVOID_COMPLETE;
          }
          break;
          
        case AVOID_COMPLETE:
          // Avoidance complete, check if clear
          Serial.println("SW_AVOID: Complete, checking if clear...");
          
          if (checkClearOfObstacles()) {
            Serial.println("STATE TRANSITION: SW_AVOIDING -> SW_WANDERING (clear!)");
            smartWanderState = SW_WANDERING;
            wanderMoving = false; // Reset wander state
            lastWanderTime = millis();
            avoidState = AVOID_TURNING; // Reset for next time
          } else {
            // Still obstacles nearby, continue avoiding
            Serial.println("SW_AVOID: Still obstacles nearby, continuing avoidance...");
            avoidState = AVOID_TURNING; // Restart avoidance
          }
          break;
      }
      break;
  }
}

// ===================================== Main Loop ===================================================
/*
  Printing helper function that prints the data as the function is called
*/
void printSensorData() {
  Serial.print("left sonar: " + String(leftSonarDist));
  Serial.print(", ");
  Serial.println("right sonar: "+ String(rightSonarDist));
  Serial.print("lidar F: "+String(frontLidarDist));
  Serial.print(", B: " + String(backLidarDist));
  Serial.print(", L: " + String(leftLidarDist));
  Serial.println(", R: " + String(rightLidarDist));
}
/*
  Printing helper function that prints the data periodically
*/
void printSensorDataPeriodically(){
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    printSensorData();
    lastPrint = millis();
  }
}

void setup() {
  int baudrate = 9600; //serial monitor baud rate'
  init_stepper(); //set up stepper motor

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder

  Serial.begin(baudrate);     //start serial monitor communication
  
  pinMode(frontLdr, OUTPUT);
  pinMode(backLdr, OUTPUT);
  pinMode(leftLdr, OUTPUT);
  pinMode(rightLdr, OUTPUT);
  
  Serial.println("Robot starting...Put ON TEST STAND");
  Serial.print("Behavior Mode: ");
  if (currentMode == COLLIDE_MODE) {
    Serial.println("COLLIDE (Aggressive Kid) - RED LED");
  } else if (currentMode == RUNAWAY_MODE) {
    Serial.println("RUNAWAY (Shy Kid) - YELLOW LED");
  } else if (currentMode == FOLLOW_MODE) {
    Serial.println("FOLLOW (Curious Kid) - YELLOW+GREEN LED");
  } else if (currentMode == RANDOM_WANDER) {
    Serial.println("RANDOM WANDER - GREEN LED");
  } else if (currentMode == SMART_WANDER_MODE) {
    Serial.println("SMART WANDER STATE MACHINE");
    Serial.println("  GREEN LED = Wandering");
    Serial.println("  RED LED = Collision Detected");
    Serial.println("  YELLOW LED = Avoiding Obstacle");
  }
  delay(pauseTime);
}

void loop() {
  updateSonarReadings();
  updateLidarReadings();
  
  // Robot continuously reads sensors and reacts in real-time
  if (currentMode == COLLIDE_MODE) {
    collideBehavior(); // COLLIDE: Drives forward, stops immediately when obstacle detected
  } else if (currentMode == RUNAWAY_MODE) {
    runawayBehavior(); // RUNAWAY: Sits still, runs away using potential fields when approached
  } else if (currentMode == FOLLOW_MODE) {
    followBehavior(); // FOLLOW: Curious kid follows object at target distance with proportional control
  } else if (currentMode == RANDOM_WANDER) {
    randomWander(); // RANDOM WANDER: Wanders randomly
  } else if (currentMode == SMART_WANDER_MODE){
    smartWanderStateMachine(); // SMART WANDER: State machine (wander -> collide -> avoid)
  }
  // printSensorDataPeriodically();
}

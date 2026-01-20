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
int maximumStoppingDist = 10; //cm

// Force Constants
const int MAX_FORCE = 100;
const float K_REPULSIVE = 200.0;

// Force Vector values
float forceVectorx = 0;
float forceVectory = 0;
float forceVectorAngle = 0;

// ------------------------------------- Sensors -------------------------------------------------------------------
const float ANGLE_LEFT_SONAR = 45.0;     // Left sonar at 45 degrees
const float ANGLE_RIGHT_SONAR = -45.0;   // Right sonar at -45 degrees
const float ANGLE_FRONT_LIDAR = 0.0;     // Front LIDAR at 0 degrees
const float ANGLE_BACK_LIDAR = 180.0;    // Back LIDAR at 180 degrees
const float ANGLE_LEFT_LIDAR = 90.0;     // Left LIDAR at 90 degrees
const float ANGLE_RIGHT_LIDAR = -90.0;   // Right LIDAR at -90 degrees

struct SensorPacket {
    int frontLidar;
    int backLidar;
    int leftLidar;
    int rightLidar;
    int leftSonar;
    int rightSonar;
    // This line tells the RPC library how to pack the data
    MSGPACK_DEFINE_ARRAY(frontLidar, backLidar, leftLidar, rightLidar, leftSonar, rightSonar);
};
// Sensor interval
unsigned long lastSensorRequest = 0;
const long sensorInterval = 100; // Only check sensors every 100ms
SensorPacket sensorData; // initialize the sensor packet
enum BehaviorMode { // Behavior modes
  
};
BehaviorMode currentMode = ;  // Change this to switch modes
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
// -----------------------------------------Layer 2 ------------------------------------------------------------------------
// -----------------------------------------Layer 1 ------------------------------------------------------------------------
// follow wall

// -----------------------------------------Layer 0 ------------------------------------------------------------------------

//void avoid


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
}

void loop() {
  stepperLeft.runSpeed();
  stepperRight.runSpeed();
  updateSensorData();
}

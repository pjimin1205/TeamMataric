#include <RPC.h>
#include <Arduino.h>//include for PlatformIO Ide
#include <AccelStepper.h>//include the stepper motor library
#include <MultiStepper.h>//include multiple stepper motor library

// TURN THIS ON TO SEE SENSOR READINGS
bool printSenRead = false;

// TURN THIS ON TO SEE FORCE CALCULATIONS
bool printForceCalc = true;

// Sensor interval
unsigned long lastSensorRequest = 0;
const long sensorInterval = 100; // Only check sensors every 100ms

// Orientation Interval
unsigned long lastReorientRequest = 0;
const long reorientInterval = 3000;

// Force Constants
const int MAX_FORCE = 100;
const float K_REPULSIVE = 200.0;

// Sensor directions
const float ANGLE_LEFT_SONAR = 45.0;     // Left sonar at 45 degrees
const float ANGLE_RIGHT_SONAR = -45.0;   // Right sonar at -45 degrees
const float ANGLE_FRONT_LIDAR = 0.0;     // Front LIDAR at 0 degrees
const float ANGLE_BACK_LIDAR = 180.0;    // Back LIDAR at 180 degrees
const float ANGLE_LEFT_LIDAR = 90.0;     // Left LIDAR at 90 degrees
const float ANGLE_RIGHT_LIDAR = -90.0;   // Right LIDAR at -90 degrees


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

int maximumStoppingDist = 10; //cm

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

// initialize the sensor packet

SensorPacket sensorData;

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

void randomWander(){
  if (millis() - lastReorientRequest >= reorientInterval) {
    lastReorientRequest = millis();

    stepperLeft.setSpeed(random(200, 1000));
    stepperRight.setSpeed(random(200, 1000));
  }
}

bool collideDetection(SensorPacket p){

  if(p.frontLidar < maximumStoppingDist && p.frontLidar > 0){
    //obstacle is too close return true
    digitalWrite(redLED, HIGH);
    return true;
  } else if(p.rightSonar < maximumStoppingDist && p.rightSonar > 0){
    digitalWrite(ylwLED, HIGH);
    return true;
  } else if(p.leftSonar < maximumStoppingDist && p.leftSonar > 0){
    digitalWrite(grnLED, HIGH);
    return true;
  }
  digitalWrite(grnLED, LOW);
  digitalWrite(redLED, LOW);
  digitalWrite(ylwLED, LOW);
  return false;
}

void computeRepulsiveVector(float &Fx, float &Fy) {
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
}

float repulsiveForce(int distance) {
  if (distance <= 0 || distance > maximumStoppingDist) return 0;
  // Inverse square law with safe distance
  float force = K_REPULSIVE / (distance * distance + 1);
  return constrain(force, 0, MAX_FORCE);
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

  if (printForceCalc){
  }

  if(!collideDetection(sensorData)){
    randomWander();
  } else{
    stepperLeft.setSpeed(0);
    stepperRight.setSpeed(0);
  }

  stepperLeft.runSpeed();
  stepperRight.runSpeed();
}
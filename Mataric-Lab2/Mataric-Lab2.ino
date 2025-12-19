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
// MOTOR FUNCTIONS
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
  Makes the robot wander randomly.
  The maximum distance it'll wander in one motion is 30 cm.
  The maximum angle the robot will rotate is by 180 deg to the left or right.
  
  There are no inputs.
*/
void randomWander(){
  turnOffLEDs();
  digitalWrite(greenLED, HIGH);
  randomAngle = getRandomNumber(360); // max of 360 deg
  randomDistance = getRandomNumber(30); // maximum of 30 cm
  // make robot turn other way if angle is over 180 degrees
  if(randomAngle > 180){
    randomAngle = -1*(randomAngle - 180);
  }
  goToAngle(randomAngle);
  forward(randomDistance);
}

/*
  This function generates a random number.
  This function is called by the randomWander function.

  This input is maxVal, the maximum value of the randomly generated number.
  This outputs a random number.
*/
private int getRandomNumber(int maxVal){
  randomSeed(analogRead(0)); //generate a new random number
  randNumber = random(maxVal); //uses random number
}

void setup() {
  int baudrate = 9600; //serial monitor baud rate'
  init_stepper(); //set up stepper motor

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder


  Serial.begin(baudrate);     //start serial monitor communication

  Serial.println("Robot starting...Put ON TEST STAND");
  delay(pauseTime); //always wait 2.5 seconds before the robot moves

}

void loop() {
  Serial.println("Starting loop...");
  // delay(wait_time);               //wait to move robot or read data
  // int circle_diameter_cm = 92; // 3 ft
  // int direction = 1; // left
  randomWander();
}

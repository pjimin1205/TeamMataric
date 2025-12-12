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
int leftSpd = 1000;
int rightSpd = 1000;

//define encoder pins
#define LEFT 0        //left encoder
#define RIGHT 1       //right encoder
const int ltEncoder = 18;        //left encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
const int rtEncoder = 19;        //right encoder pin (Mega Interrupt pins 2,3 18,19,20,21)
volatile long encoder[2] = {0, 0};  //interrupt variable to hold number of encoder counts (left, right)
int lastSpeed[2] = {0, 0};          //variable to hold encoder speed (left, right)
int accumTicks[2] = {0, 0};         //variable to hold accumulated ticks since last reset

// Constant Vars
#define INCHES_TO_CM 2.54
#define TWO_FEET_IN_STEPS 1848
#define WIDTH_OF_BOT_CM 21.2
#define WHEEL_DIAM_CM 8.4
#define STEPS_PER_ROT 800
#define CM_PER_ROTATION 26.39
#define CM_TO_STEPS_CONV STEPS_PER_ROT/CM_PER_ROTATION
#define ENCODER_TICKS_PER_ROTATION 20

// Helper Functions

//interrupt function to count left encoder tickes
void LwheelSpeed() {
  encoder[LEFT] ++;  //count the left wheel encoder interrupts
}

//interrupt function to count right encoder ticks
void RwheelSpeed() {
  encoder[RIGHT] ++; //count the right wheel encoder interrupts
}

void allOFF(){
  for (int i = 0;i<3;i++){
    digitalWrite(leds[i],LOW);
  }
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

/*This function, runToStop(), will run the robot until the target is achieved and
   then stop it
*/
void runToStop ( void ) {
  Serial.println("Run to stop");
  int runNow = 1;
  int rightStopped = 0;
  int leftStopped = 0;

  while (runNow) {
    if (!stepperRight.run()) {
      rightStopped = 1;
      stepperRight.stop();//stop right motor
    }
    if (!stepperLeft.run()) {
      leftStopped = 1;
      stepperLeft.stop();//stop ledt motor
    }
    if (rightStopped && leftStopped) {
      runNow = 0;
    }
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

/*
  direction is either -1 or 1
  circle left is 1
  circle right is -1
*/

void moveCircle(int diam, int direction){
  turnOffLEDs(); // turn off all LEDs
  digitalWrite(redLED, HIGH); // turn on red LED

  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

  Serial.println("Moving in a circle...");

  double totalOuter = (double)diam + WIDTH_OF_BOT_CM; //Outer wheel path diameter in cm
  double totalInner = (double)diam - WIDTH_OF_BOT_CM; //Inner wheel path diameter in cm

  double outerCirc = totalOuter * PI; //Circumference of outerwheel path in cm
  double innerCirc = totalInner * PI; //Circumference of innerwheel path in cm

  int numStepsOuter = outerCirc * CM_TO_STEPS_CONV; //turn circumference to steps
  int numStepsInner = innerCirc * CM_TO_STEPS_CONV;
  Serial.println("Outer steps ="); Serial.print(" "); Serial.print(numStepsOuter);
  Serial.println("Inner steps ="); Serial.print(" "); Serial.print(numStepsInner);

  int outerWheelSpd = 400; //steps per sec
  // 1000 speed is possible, results in inaccuracy

  double timeToComplete = (double)numStepsOuter/outerWheelSpd; //time it will take the outer wheel to complete path in sec. Needs to be a double so they have almost exact same time
  Serial.println("Time to complete this circle ="); Serial.print(" "); Serial.print(timeToComplete);

  double innerWheelSpd = (double)numStepsInner/timeToComplete; //matching up the completion times by setting inner wheel speed
  Serial.println("Inner Wheel Speed ="); Serial.print(" "); Serial.print(innerWheelSpd);

  long positions[2]; // Array of desired stepper positions

  if(direction > 0){
  Serial.println("Circling left");
  positions[0] = numStepsOuter; //right motor absolute position
  positions[1] = numStepsInner; //left motor absolute position
  steppers.moveTo(positions);
  stepperLeft.setSpeed(innerWheelSpd);//set left motor speed
  stepperRight.setSpeed(outerWheelSpd);//set right motor speed
  } else{
  Serial.println("Circling right");
  positions[0] = numStepsInner; //right motor absolute position
  positions[1] = numStepsOuter; //left motor absolute position
  steppers.moveTo(positions);
  stepperLeft.setSpeed(outerWheelSpd);//set left motor speed
  stepperRight.setSpeed(innerWheelSpd);//set right motor speed
  }

  steppers.runSpeedToPosition();
  turnOffLEDs();
}

void moveFigure8(int diam, int direction){
  turnOffLEDs();
  digitalWrite(redLED, HIGH); digitalWrite(yellowLED, HIGH);

  moveCircle(diam, direction);
  moveCircle(diam, -direction);
  turnOffLEDs();
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
  /*
  //check if both encoders have traveled the correct distance
  int currentLeftTicks = encoder[LEFT]; //how many encoder ticks have passed by
  int currentRightTicks = encoder[RIGHT];

  int currentLeftSteps = encoderTicksToSteps(currentLeftTicks); //how many steps that have contributed to movement have passed 
  int currentRightSteps = encoderTicksToSteps(currentRightTicks);

  Serial.println(String("leftTicks:   ")+currentLeftTicks+String(" ticks"));
  Serial.println(String("rightTicks:  ")+currentRightTicks+String(" ticks"));


  int leftStepsToGo = numSteps - currentLeftSteps;
  int rightStepsToGo = numSteps - currentRightSteps;

  Serial.println(String("leftStepsToGo:   ")+leftStepsToGo+String(" steps"));
  Serial.println(String("rightStepsToGo:  ")+rightStepsToGo+String(" steps"));

  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

  if(spinSpeed > 0){
    positions[1] = -leftStepsToGo; //left motor position
    positions[0] = rightStepsToGo; //right motor position
    steppers.moveTo(positions);
  } else {
    positions[1] = leftStepsToGo; //left motor position
    positions[0] = -rightStepsToGo; //right motor position
    steppers.moveTo(positions);
  }

  stepperLeft.setSpeed(-spinSpeed);//set left motor speed
  stepperRight.setSpeed(spinSpeed);//set right motor speed

  steppers.runSpeedToPosition(); // Blocks until all are in position
  */
  turnOffLEDs();
}

/*
  Helper function for goToGoal
  Provides the error between the expected distance and actual distance.
  oves the robot by tthis error amount.

  input di tan ce i.int s in CMENTIMETERS. e
, th, the forward component distance. ecnatsid lautca dna ecnatsid detcepxe eht neew
goToGoalErrorCorrection(side);*/
void goToGoalErrorCorrection(int distance){
  // get actual encoder distance
  int actual_steps_left = encoderTicksToSteps(encoder[LEFT]);
  int actual_steps_right = encoderTicksToSteps(encoder[RIGHT]);
  // get difference
  int distance_step = distance*CM_TO_STEPS_CONV; // taken from forward function
  int diff_left = distance_step - actual_steps_left;
  int diff_right = distance_step - actual_steps_right;
  Serial.println(String("diff left:   ")+diff_left+String(" steps"));
  Serial.println(String("diff right:  ")+diff_left+String(" steps"));

  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  stepperLeft.moveTo(diff_left);
  stepperRight.moveTo(diff_right);
  int l_direction = 1;
  int r_direction = 1;
  if(diff_left < 0){
    l_direction = -1;
  }
  if(diff_right < 0){
    r_direction = -1;
  }
  Serial.println(String("l direction:  ")+l_direction);
  Serial.println(String("r direction:  ")+r_direction);
  stepperLeft.setSpeed(l_direction*200);
  stepperRight.setSpeed(r_direction*200);

  steppers.runSpeedToPosition();
}

/*
  Turn robot to an angle, go forward to a particular direction.
  Input x value is in centimeters.
  Input y value is in centimeters.

  Accounts for odometry error.
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
  double forwardDistance_ft = forwardDistance_cm / 30.48;
  Serial.println(String("  going forward by ") + forwardDistance_ft + (" ft"));
  forward(forwardDistance_cm);
  
  Serial.println("Starting goToGoal Error Correction...");
  delay(1000);
  // goToGoalErrorCorrection(forwardDistance_cm);

  Serial.println("Turning off led's for goal");
  turnOffLEDs();
}

/*
  Input is the number of encoder ticks
*/
int encoderTicksToSteps(int ticks){
  int steps = ticks*STEPS_PER_ROT/ENCODER_TICKS_PER_ROTATION;
  Serial.println(String("TickstoSteps:  ")+steps+String(" steps"));
  return (steps);
}
/*
  Move the robot in a square shape.
  Input is length of the sides in CENTIMETERS
*/
void moveSquare(int side){
  turnOffLEDs();
  encoder[LEFT] = 0;                          //clear the left encoder data buffer
  encoder[RIGHT] = 0;
  Serial.println(String("move square: ") + side + "cm");
  digitalWrite(redLED, HIGH); digitalWrite(greenLED, HIGH); digitalWrite(yellowLED, HIGH);
  int indicate_move_square = 2000;
  delay(indicate_move_square);
  goToGoal(side, 0);
  goToGoal(0, -side);
  goToAngle(-90);
  // TODO: add in encoder functionality
  
  turnOffLEDs();
}

/*
  direction is either -1 or 1
  pivot left is 1
  pivot right is -1
*/
void pivot(int direction) {
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

  long positions[2]; // Array of desired stepper positions

  Serial.println("Pivot");
  double angle = PI/2;
  int numSteps = angle*WIDTH_OF_BOT_CM*CM_TO_STEPS_CONV;

 if(direction > 0){
  Serial.println("Pivot Left");
  positions[0] = numSteps; //right motor absolute position
  positions[1] = 0; //left motor absolute position
  steppers.moveTo(positions);
  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed
  } else{
  Serial.println("Pivot Right");
  positions[0] = 0; //right motor absolute position
  positions[1] = numSteps; //left motor absolute position
  steppers.moveTo(positions);
  stepperLeft.setSpeed(leftSpd);//set left motor speed
  stepperRight.setSpeed(rightSpd);//set right motor speed
  }
  steppers.runSpeedToPosition();
}

/*
  Direction is either -1 or 1.
  1 is to the left.
  -1 is to the right.
*/
void spin(int direction) {
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  double angle = PI/2;

  Serial.println("Spin");
  //int numSteps = angle / minIntervalAngle;
  int numSteps = angle*WIDTH_OF_BOT_CM/2*CM_TO_STEPS_CONV;
  long positions[2]; // Array of desired stepper positions
  positions[0] = direction*numSteps; //right motor absolute position
  positions[1] = -direction*numSteps; //left motor absolute position
  steppers.moveTo(positions);

  stepperLeft.setSpeed(-direction*leftSpd);//set left motor speed
  stepperRight.setSpeed(direction*rightSpd);//set right motor speed

  steppers.runSpeedToPosition(); // Blocks until all are in position
}

/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
  Direction is -1 or 1
  Turn left is 1
  Turn right is -1
*/
void turn(int direction) {
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

  Serial.println("Turn");
  int numSteps = 2000;
  int leftSpeed = 2*leftSpd;
  int rightSpeed = 2*rightSpd;

  long positions[2]; // Array of desired stepper positions
  if(direction > 0){
  Serial.println("Turning Left");
  positions[0] = 2*numSteps; //right motor absolute position
  positions[1] = numSteps; //left motor absolute position
  steppers.moveTo(positions);
  stepperLeft.setSpeed(leftSpeed);//set left motor speed
  stepperRight.setSpeed(rightSpeed*2);//set right motor speed
  } else{
  Serial.println("Turning Right");
  positions[0] = numSteps; //right motor absolute position
  positions[1] = 2*numSteps; //left motor absolute position
  steppers.moveTo(positions);
  stepperLeft.setSpeed(leftSpeed*2);//set left motor speed
  stepperRight.setSpeed(rightSpeed);//set right motor speed
  }

  steppers.runSpeedToPosition(); // Blocks until all are in position
}
/*
  input distance is in centimeters
*/
void forward(int distance) {
  Serial.println("Forward");
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);
  int distance_step = distance*CM_TO_STEPS_CONV;
  stepperLeft.moveTo(distance_step);//left motor absolute position
  stepperRight.moveTo(distance_step);//right motor absolute position
  stepperLeft.setSpeed(200);
  stepperRight.setSpeed(200);

  steppers.runSpeedToPosition();
}
/*
  INSERT DESCRIPTION HERE, what are the inputs, what does it do, functions used
*/
void reverse(int distance) {
  Serial.println("Reverse");
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

  stepperLeft.moveTo(-distance);//left motor absolute position
  stepperRight.moveTo(-distance);//right motor absolute position
  stepperLeft.setSpeed(-200);
  stepperRight.setSpeed(-200);

  steppers.runSpeedToPosition();
}
/*
  Stops the stepper motors.
*/
void stop() {
  Serial.println("Stop");
  stepperLeft.stop();
  stepperRight.stop();
}
/*
  used in calibration of left motor to map steps to wheel rotations

*/
void leftMotorCal() {
  Serial.println("Left Motor Calibration");
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

  stepperRight.moveTo(0);
  stepperRight.setSpeed(0);

  stepperLeft.moveTo(800);
  stepperLeft.setSpeed(100);

  steppers.runSpeedToPosition();

}
/*
  used in calibration of left motor to map steps to wheel rotations

*/
void rightMotorCal() {
  Serial.println("Right Motor Calibration");
  stepperLeft.setCurrentPosition(0);
  stepperRight.setCurrentPosition(0);

  stepperLeft.moveTo(0);
  stepperLeft.setSpeed(0);

  stepperRight.moveTo(800);
  stepperRight.setSpeed(100);

  steppers.runSpeedToPosition();

}

void setup() {
  int baudrate = 9600; //serial monitor baud rate'
  init_stepper(); //set up stepper motor

  attachInterrupt(digitalPinToInterrupt(ltEncoder), LwheelSpeed, CHANGE);    //init the interrupt mode for the left encoder
  attachInterrupt(digitalPinToInterrupt(rtEncoder), RwheelSpeed, CHANGE);   //init the interrupt mode for the right encoder


  Serial.begin(baudrate);     //start serial monitor communication

  Serial.println("Robot starting...Put ON TEST STAND");
  delay(pauseTime); //always wait 2.5 seconds before the robot moves
  
  // Lab 1 Basic functionality demo
  int demo1PauseTime = 1000;
  delay(demo1PauseTime);
  Serial.println(WIDTH_OF_BOT_CM);
  /*
  forward(TWO_FEET_IN_STEPS);
  delay(demo1PauseTime);
  reverse(TWO_FEET_IN_STEPS);
  delay(demo1PauseTime);
  spin(1); // spins left
  delay(demo1PauseTime);
  spin(-1); // spins right
  delay(demo1PauseTime);
  turn(1);
  delay(demo1PauseTime);
  turn(-1);
  delay(demo1PauseTime);
  pivot(1); // left
  delay(demo1PauseTime);
  pivot(-1); // right
  delay(demo1PauseTime);
  stop();
  */
  // end of basic functionality demo

}

void loop() {
  // put your main code here, to run repeatedly:
  print_encoder_data();   //prints encoder data
  Serial.println("Starting loop...");
  delay(wait_time);               //wait to move robot or read data
  int circle_diameter_cm = 92; // 3 ft
  int direction = 1; // left
  
  //Begin Loop Demo
  // circle
  /*
  moveCircle(circle_diameter_cm, direction); // turn left
  delay(wait_time);
  */
  
  // figure 8
  // moveFigure8(circle_diameter_cm, direction); // start left
  // delay(wait_time);

  // go to angle
  // int angle_deg = 53;
  // goToAngle(angle_deg);
  // delay(wait_time);
  // goToAngle(-angle_deg);
  // delay(wait_time);

  // go to goal
  // float angle_rad = 53* (PI/180);
  // float distance_cm = 152.4; // 3 feet
  // int x_cm = cos(angle_rad)*distance_cm;
  // int y_cm = sin(angle_rad)*distance_cm;
  // goToGoal(x_cm, y_cm);

  // goToGoal(-61, -61);
  
  // square
  delay(wait_time*2);
  int side_length_cm = 92; // 3 ft
  moveSquare(side_length_cm);
  
}

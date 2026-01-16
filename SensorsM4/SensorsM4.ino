#include <RPC.h>
#include "Arduino.h"

#define NUM_SENSORS 3

int sensorResults[NUM_SENSORS];

int numOfSamples = 3;

//Lidar Sensor Pins
#define frontLdr 8
#define backLdr 9
#define leftLdr 10
#define rightLdr 12

//Sonar Sensor Pins
#define leftSnrTrigPin 4
#define leftSnrEchoPin 11
#define rightSnrTrigPin 3
#define rightSnrEchoPin 2

#define delayUs 20  //delay betwen readings

float duration, distance;

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

int front, back, left, right;
int leftSon, rightSon;

// Reading sensor functions
int read_lidar(int pin) {
  int d;
  int16_t t = pulseIn(pin, HIGH);
  d = (t - 1000) * 3 / 40;
  if (t == 0 || t > 1850 || d < 0) { d = 0; }
  return d;
}

int read_sonar(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  return(distance);
}

// int read_sonar(int pin) {
//   float velocity((331.5 + 0.6 * (float)(20)) * 100 / 1000000.0);
//   uint16_t distance, pulseWidthUs;
//   pinMode(pin, OUTPUT);
//   digitalWrite(pin, LOW);
//   digitalWrite(pin, HIGH);            //Set the trig pin High
//   delayMicroseconds(10);              //Delay of 10 microseconds
//   digitalWrite(pin, LOW);             //Set the trig pin Low
//   pinMode(pin, INPUT);                //Set the pin to input mode
//   pulseWidthUs = pulseIn(pin, HIGH);  //Detect the high level time on the echo pin, the output high level time represents the ultrasonic flight time (unit: us)
//   distance = pulseWidthUs * velocity / 2.0;
//   if (distance < 0 || distance > 50) { distance = 0; }
//   return distance;
// }

// int avgSonar(int pin, int num) {
//   int sum = 0;
//   int data;
//   int distance;
//   for (int i = 0; i < num; i++) {
//     data = read_sonar(pin);
//     sum = sum + data;
//     delayMicroseconds(delayUs);  //give ADC time to recharge
//   }
//   return distance = sum / num;
// }


SensorPacket getSensorData() {
 SensorPacket data;
    data.frontLidar = front;   // Replace with your sensor reads later
    data.backLidar = back;
    data.leftLidar = left;
    data.rightLidar = right;
    data.leftSonar = leftSon;
    data.rightSonar = rightSon;
    return data;
}

void setup() {
  RPC.begin();
  // Give the system time to stabilize before binding
  pinMode(frontLdr, OUTPUT);
  pinMode(backLdr, OUTPUT);
  pinMode(leftLdr, OUTPUT);
  pinMode(rightLdr, OUTPUT);
  
  delay(500);
  RPC.bind("getSensorData", getSensorData);
  
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, LOW); // Red ON
}

void loop() {
  front = read_lidar(frontLdr);
  back = read_lidar(backLdr);
  left = read_lidar(leftLdr);
  right = read_lidar(rightLdr);
  leftSon = read_sonar(leftSnrTrigPin, leftSnrEchoPin);
  rightSon = read_sonar(rightSnrTrigPin, rightSnrEchoPin);
}




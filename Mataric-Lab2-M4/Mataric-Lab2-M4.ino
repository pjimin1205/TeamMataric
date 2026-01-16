#include "Arduino.h"
#include "RPC.h"

// co processor
// server
//state LEDs connections
#define redLED 5            //red LED for displaying states
#define greenLED 6            //green LED for displaying states
#define grnLED 6            //green LED for displaying states
#define yellowLED 7            //yellow LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED

struct LidarData {
  int front;
  int back;
  int left;
  int right;
};

struct SonarData {
  int left;
  int right;
};

void setup() {
  RPC.begin();
  Serial.begin(9600);
  RPC.bind("read_lidars", read_lidars);
  RPC.bind("read_sonars", read_sonars);
}

void loop() {
  dist.front = read_lidar(frontLdr);
}

int sensorRead(){
  int result = analogRead(A0);
  return result;
}

LidarData read_lidars() {
  LidarData data;
  data.front = analogRead(FRONT_LIDAR);
  data.back = analogRead(BACK_LIDAR);
  data.left = analogRead(LEFT_LIDAR);
  data.right = analogRead(RIGHT_LIDAR);
  return data;
}

SonarData read_sonars() {
  SonarData data;
  data.left = analogRead(LEFT_SONAR);
  data.right = analogRead(RIGHT_SONAR);
  return data;
}

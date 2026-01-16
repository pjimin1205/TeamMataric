#include "Arduino.h"
#include "RPC.h"

// main core
// client

//state LEDs connections
#define redLED 5            //red LED for displaying states
#define greenLED 6            //green LED for displaying states
#define grnLED 6            //green LED for displaying states
#define yellowLED 7            //yellow LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED

void setup() {
  RPC.begin(); // boots M4
  Serial.begin(9600);
  delay(100);
}

void loop() {
  dist.front = read_lidar(frontLdr);
  dist.back = read_lidar(backLdr);
  dist.left = read_lidar(leftLdr);
  dist.right = read_lidar(rightLdr);
  dist2.right = read_sonar(rightSnr);
  dist2.left = read_sonar(leftSnr);

  RPC.println("front: "+ String(dist.front) +
  ", back: "+ String(dist.back) + 
  ", left: "+ String(dist.left) +
  ", right: "+ String(dist.right));

  RPC.println("front left: "+String(dist2.left) +
  ", front right: "+ String(dist2.right));

  struct lidar data = RPC.call("read_lidars").as<struct lidar>();
  struct sonar data2 = RPC.call("read_sonars").as<struct sonar>();
  RPC.println("front: "+ String(data.front) +
  ", back: "+ String(data.back) +
  ", left: "+ String(data.left) +
  ", right: "+ String(data.right));
}
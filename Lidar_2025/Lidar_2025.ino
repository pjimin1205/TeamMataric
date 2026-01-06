//Lidar_2025.ino
//CAB 12.19.2025
//this code reads and test 4 Lidar on robot with out using Dual Core on Giga

#include "Arduino.h"

#define frontLdr 8
#define backLdr 9
#define leftLdr 10
#define rightLdr 12

void setup() {
  Serial.begin(9600);
  pinMode(frontLdr, OUTPUT);
  pinMode(backLdr, OUTPUT);
  pinMode(leftLdr, OUTPUT);
  pinMode(rightLdr, OUTPUT);
  delay(1000);
  Serial.println("robot starting...");
}

void loop() {
  int front, back, left, right;
  front = read_lidar(frontLdr);
  back = read_lidar(backLdr);
  left = read_lidar(leftLdr);
  right = read_lidar(rightLdr);
  Serial.print("f: ");
  Serial.print(front);
  Serial.print(" b: ");
  Serial.print(back);
  Serial.print(" l: ");
  Serial.print(left);
  Serial.print(" r: ");
  Serial.println(right);
  delay(100);
}

// reads a lidar given a pin
int read_lidar(int pin) {
  int d;
  int16_t t = pulseIn(pin, HIGH);
  d = (t - 1000) * 3 / 40;
  if (t == 0 || t > 1850 || d < 0) { d = 0; }
  return d;
}

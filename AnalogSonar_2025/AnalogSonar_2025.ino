//AnalogSonar_2025.ino
//C.A.Berry 12.18.25
//Read the analog sonar without multicore on the Arduino Giga

#include "Arduino.h"

#define leftSnr 4
#define rightSnr 3
#define baudrate 9600
#define num_of_samples 5
#define delayUs 20  //delay betwen readings

void setup() {
  Serial.begin(baudrate);
  delay(1000);
  Serial.println("robot starting....");
}

void loop() {
  int leftDist, rightDist;
  int leftAvg, rightAvg;
  leftDist = read_sonar(leftSnr);
  delayMicroseconds(delayUs);
  rightDist = read_sonar(rightSnr);
  Serial.print("left  = ");
  Serial.print(leftDist);
  Serial.print("\trightDist = ");
  Serial.println(rightDist);
  delayMicroseconds(delayUs);  //give ADC time to recharge

  // leftAvg = avgSonar(leftSnr, num_of_samples);
  // rightAvg = avgSonar(rightSnr, num_of_samples);
  // Serial.print("leftAvG  = ");
  // Serial.print(leftAvg);
  // Serial.print("\trightAvg = ");
  // Serial.println(rightAvg);

  delay(100);//make print out readable
}

//average n number of sonar readings
int avgSonar(int pin, int num) {
  int sum = 0;
  int data;
  int distance;
  for (int i = 0; i < num; i++) {
    data = read_sonar(pin);
    sum = sum + data;
    delayMicroseconds(delayUs);  //give ADC time to recharge
  }
  return distance = sum / num;
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

#include "Arduino.h"
#define lidarPin 8 // front lidar

volatile unsigned long pulseStart = 0;
volatile unsigned long pulseEnd = 0;
volatile bool newDataReady = false;

void setup() {
    Serial.begin(9600);
    pinMode(lidarPin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(lidarPin), lidarISR, CHANGE);
}

void lidarISR() {
  if (digitalRead(lidarPin) == HIGH) {
    pulseStart = micros();
  } else {
    pulseEnd = micros();
    newDataReady = true;
  }
}

void loop() {
  if (newDataReady) {
    int distance = (pulseEnd - pulseStart) / 10;
    newDataReady = false;
  }
  Serial.println("curr value: " + String(pulseEnd-pulseStart));
}
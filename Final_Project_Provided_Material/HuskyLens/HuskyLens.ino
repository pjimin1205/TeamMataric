/*
Code to track a color for homing and docking the robot. Use the red robot box top to train the color

*/

#include <HUSKYLENS.h>
#include "Wire.h"

HUSKYLENS huskylens;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  while (!huskylens.begin(Wire)) {
    Serial.println("HuskyLens not detected!");
    delay(1000);
  }

  Serial.println("HuskyLens ready!");
}

void loop() {
  if (!huskylens.request()) {
    Serial.println("Request failed");
    return;
  }

  if (!huskylens.isLearned()) {
    Serial.println("Nothing learned yet");
    return;
  }

  if (!huskylens.available()) {
    Serial.println("No object detected");
  } else {
    while (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();

      Serial.print("ID: ");
      Serial.print(result.ID);
      Serial.print("  X: ");
      Serial.print(result.xCenter);
      Serial.print("  Y: ");
      Serial.print(result.yCenter);
      Serial.print("  Width: ");
      Serial.print(result.width);
      Serial.print("  Height: ");
      Serial.println(result.height);
    }
  }

  delay(200);
}

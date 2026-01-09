#include "Arduino.h"
#include "RPC.h"

//state LEDs connections
#define redLED 5            //red LED for displaying states
#define greenLED 6            //green LED for displaying states
#define grnLED 6            //green LED for displaying states
#define yellowLED 7            //yellow LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED

void setup() {
  RPC.begin(); // boots M4
  pinMode(redLED, OUTPUT);
  digitalWrite(redLED, HIGH);
}

void loop() {
}

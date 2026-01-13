#include <RPC.h>

int getDist() {
  return 42;
}

void setup() {
  RPC.begin();
  // Give the system time to stabilize before binding
  delay(500);
  RPC.bind("getDist", getDist);
  
  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, LOW); // Red ON
}

void loop() {
  // Never leave an M4 loop totally empty; it can cause OS jitters
  delay(100); 
}
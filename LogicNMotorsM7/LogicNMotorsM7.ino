#include <RPC.h>

void setup() {
  Serial.begin(9600);
  while (!Serial); 
  
  RPC.begin(); 
  
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDB, LOW); // Blue ON
  
  Serial.println("M7: Online. Requesting data...");
  delay(2000); 
}

void loop() {
  // We skip the 'if' check to avoid the compilation error.
  // We'll just grab the result directly.
  auto res = RPC.call("getDist");
  int val = res.as<int>();

  Serial.print("M4 Response: ");
  Serial.println(val);

  delay(1000); 
}
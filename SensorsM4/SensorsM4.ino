#include <RPC.h>
#include "Arduino.h"

/*
  SensorsM4.ino
  --------------
  Purpose:
    Run on the M4 core to read range sensors (LIDAR and ultrasonic
    sonars) and provide a serialized `SensorPacket` via RPC to the
    M7 core. This file contains low-level sensor reading routines
    and minimal data aggregation.

  Hardware / Pins:
    - Front LIDAR: pin 8
    - Back  LIDAR: pin 9
    - Left  LIDAR: pin 10
    - Right LIDAR: pin 12
    - Left Sonar trig/echo: pins 4 / 11
    - Right Sonar trig/echo: pins 3 / 2

  Notes:
    - `RPC.bind("getSensorData", getSensorData)` exposes the
      `getSensorData` function to the M7 core; it returns a
      `SensorPacket` that matches the M7-side definition.
    - This file focuses on readable, documented sensor helpers
      without altering sensor logic.
*/

// Number of sensors (placeholder for arrays if needed)
#define NUM_SENSORS 3

// Simple buffer for potential aggregated results (unused currently)
int sensorResults[NUM_SENSORS];

// Number of samples used for averaging (if averaging added)
int numOfSamples = 3;

// LIDAR sensor pins (digital pulse input)
#define frontLdr 8
#define backLdr 9
#define leftLdr 10
#define rightLdr 12

// Sonar (HC-SR04 style) pins: TRIG -> output, ECHO -> input
#define frontLeftSnrTrigPin 4
#define frontLeftSnrEchoPin 11
#define frontRightSnrTrigPin 3
#define frontRightSnrEchoPin 2

#define backLeftSnrTrigPin 40
#define backLeftSnrEchoPin 41
#define backRightSnrTrigPin 42
#define backRightSnrEchoPin 43

#define delayUs 20  // microsecond delay between rapid readings

// Temporary globals for pulse/sonar calculations
float duration, distance;

// RPC-able sensor data packet. Mirrors the structure expected by M7.
struct SensorPacket {
    int frontLidar;
    int backLidar;
    int leftLidar;
    int rightLidar;
    int frontLeftSonar;
    int frontRightSonar;
    int backLeftSonar;
    int backRightSonar;
    // This macro tells the RPC/MsgPack layer how to serialize fields
    MSGPACK_DEFINE_ARRAY(frontLidar, backLidar, leftLidar, rightLidar, frontLeftSonar, frontRightSonar, backLeftSonar, backRightSonar);
};

// Current sensor readings (populated each loop)
int front, back, left, right;
int frontLeftSon, frontRightSon, backLeftSon, backRightSon;

/*
  read_lidar(pin)
  ----------------
  Read a (short) pulse-width-based LIDAR sensor connected to `pin`.
  Returns an integer distance in centimeters, or 0 if reading is
  invalid/out of range.
*/
int read_lidar(int pin) {
  int d;
  int16_t t = pulseIn(pin, HIGH);
  d = (t - 1000) * 3 / 40;
  if (t == 0 || t > 1850 || d < 0) { d = 0; }
  return d;
}

/*
  read_sonar(trigPin, echoPin)
  -----------------------------
  Trigger an ultrasonic sensor and measure echo time to estimate
  distance. Returns distance in centimeters. This implementation
  assumes a standard HC-SR04 timing and uses globals `duration`
  and `distance` for intermediate values.
*/
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

/*
  getSensorData()
  ----------------
  Package the most-recent sensor readings into a `SensorPacket`
  and return it. This function is bound to RPC and invoked remotely
  by the M7 core.
*/
SensorPacket getSensorData() {
  SensorPacket data;
  data.frontLidar = front;
  data.backLidar = back;
  data.leftLidar = left;
  data.rightLidar = right;
  data.frontLeftSonar = frontLeftSon;
  data.frontRightSonar = frontRightSon;
  data.backLeftSonar = backLeftSon;
  data.backRightSonar = backRightSon;
  return data;
}

/*
  setup()
  -------
  Initialize RPC, configure sensor pins, and bind the `getSensorData`
  RPC method so the M7 core can request sensor snapshots.
*/
void setup() {
  RPC.begin();
  // Configure LIDAR pins as inputs--pulseIn will read digital pulses
  pinMode(frontLdr, OUTPUT);
  pinMode(backLdr, OUTPUT);
  pinMode(leftLdr, OUTPUT);
  pinMode(rightLdr, OUTPUT);

  // Configure sonar trigger pins as OUTPUT and echo pins as INPUT
  pinMode(frontLeftSnrTrigPin, OUTPUT);
  pinMode(frontLeftSnrEchoPin, INPUT);
  pinMode(frontRightSnrTrigPin, OUTPUT);
  pinMode(frontRightSnrEchoPin, INPUT);

  pinMode(backLeftSnrTrigPin, OUTPUT);
  pinMode(backLeftSnrEchoPin, INPUT);
  pinMode(backRightSnrTrigPin, OUTPUT);
  pinMode(backRightSnrEchoPin, INPUT);

  delay(500); // allow sensors and buses to stabilize
  RPC.bind("getSensorData", getSensorData);

  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, LOW); // Red ON to indicate M4 online
}

/*
  loop()
  ------
  Periodically read each sensor and store values in global vars.
  The `getSensorData` RPC handler uses those globals to build packets.
*/
void loop() {
  front = read_lidar(frontLdr);
  back = read_lidar(backLdr);
  left = read_lidar(leftLdr);
  right = read_lidar(rightLdr);
  frontLeftSon = read_sonar(frontLeftSnrTrigPin, frontLeftSnrEchoPin);
  frontRightSon = read_sonar(frontRightSnrTrigPin, frontRightSnrEchoPin);
  backLeftSon = read_sonar(backLeftSnrTrigPin, backLeftSnrEchoPin);
  backRightSon = read_sonar(backRightSnrTrigPin, backRightSnrEchoPin);
}




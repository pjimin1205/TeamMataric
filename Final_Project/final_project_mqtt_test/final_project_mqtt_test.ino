#include <RPC.h>
#include <Arduino.h>//include for PlatformIO Ide

#include <ArduinoMqttClient.h>
#include <WiFi.h>

// ------------------------------------- LED's -------------------------------------------------------------------------
#define redLED 5            //red LED for displaying states
#define greenLED 6            //green LED for displaying states
#define grnLED 6            //green LED for displaying states
#define yellowLED 7            //yellow LED for displaying states
#define ylwLED 7            //yellow LED for displaying states
#define enableLED 13        //stepper enabled LED
int leds[3] = {5,6,7};      //array of LED pin numbers

char ssid[] = "RHIT-OPEN";        // your network SSID
char pass[] = "";    // your network password

//create the objects
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// ------------------------------------- MQTT setup -------------------------------------------------------------------------
//define broker, port and topic
// const char broker[] = "mosquitto.csse.rose-hulman.edu";
const char broker[] = "broker.hivemq.com";
int port = 1883;
const char topicSubscribe[]  = "ece445/parkj10/to_arduino";
const char topicPublish[]  = "ece445/parkj10/to_matlab";
const char sensorsTopicPublish[]  = "ece445/parkj10/sensors_to_matlab";
unsigned long previousMillis = 0;
const long sensorPublishInterval = 200; // ms interval between sensor data publishes

void setupMQTTConnection(){
  // attempt to connect to Wifi network:
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    Serial.print(".");
    delay(5000);
  }

  Serial.println("You're connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }
  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  // set the message receive callback
  mqttClient.onMessage(onMqttMessage);
  
  Serial.print("Subscribing to topic: ");
  Serial.println(topicSubscribe);
  Serial.println();

  // subscribe to a topic
  mqttClient.subscribe(topicSubscribe);
  Serial.print("Topic: ");
  Serial.println(topicSubscribe);
  Serial.println();

  // Serial.print("Subscription status: ");
  // Serial.println(mqttClient.subscribed(topicSubscribe) ? "SUCCESS" : "FAILED");
}

void onMqttMessage(int messageSize) {
  String message = "";
  while (mqttClient.available()) {
    message += (char)mqttClient.read(); // Read incoming command [5]
  }

  // Basic logic to control LEDs based on message content
  if (message == "RED_ON") {
    digitalWrite(5, HIGH);
    sendMessage("RED LED turned ON");
  }
  if (message == "RED_OFF") {
    digitalWrite(5, LOW);
    sendMessage("RED LED turned OFF");
  }
  if (message == "GREEN_ON") {
    digitalWrite(6, HIGH);
    sendMessage("GREEN LED turned ON");
  }
  if (message == "GREEN_OFF") {
    digitalWrite(6, LOW);
    sendMessage("GREEN LED turned OFF");
  }
  if (message == "YELLOW_ON") {
    digitalWrite(7, HIGH);
    sendMessage("YELLOW LED turned ON");
  }
  if (message == "YELLOW_OFF") {
    digitalWrite(7, LOW);
    sendMessage("YELLOW LED turned OFF");
  }
  Serial.println("recieved message: "+String(message));
}

void sendMessage(String message){
    // print to serial monitor
    Serial.print("Sending message:  ");
    Serial.print(message); Serial.print(" |");
    Serial.print(" to topic:  "); Serial.println(topicPublish);

    //publish the message to the specific topic
    mqttClient.beginMessage(topicPublish);
    mqttClient.print(message);
    mqttClient.endMessage();
}
// ------------------------------------- SENSOR DATA -------------------------------------------------------------------------
struct SensorPacket {
    int frontLidar;
    int backLidar;
    int leftLidar;
    int rightLidar;
    int frontLeftSonar;
    int frontRightSonar;
    int backLeftSonar;
    int backRightSonar;
    // This line tells the RPC library how to pack the data
    MSGPACK_DEFINE_ARRAY(frontLidar, backLidar, leftLidar, rightLidar, frontLeftSonar, frontRightSonar, backLeftSonar, backRightSonar);
};

unsigned long lastSensorRequest = 0;
const long sensorPollInterval = 100; // Query sensors every 100 ms
SensorPacket sensorData;
bool printSenRead = false; // for debug

void updateSensorData(){
  // Only talk to the M4 every 100ms
  if (millis() - lastSensorRequest >= sensorPollInterval) {
    lastSensorRequest = millis();
    auto res = RPC.call("getSensorData");
    sensorData = res.as<SensorPacket>();

    if(printSenRead){
      Serial.print("F: "); Serial.print(sensorData.frontLidar);
      Serial.print(" | B: "); Serial.print(sensorData.backLidar);
      Serial.print(" | L: "); Serial.print(sensorData.leftLidar);
      Serial.print(" | R: "); Serial.print(sensorData.rightLidar);
      Serial.print(" | LSonar: "); Serial.print(sensorData.frontLeftSonar);
      Serial.print(" | RSonar: "); Serial.print(sensorData.frontRightSonar);
      Serial.print(" | BLSonar: "); Serial.print(sensorData.backLeftSonar);
      Serial.print(" | BRSonar: "); Serial.print(sensorData.backRightSonar);
      Serial.println();
    }
  }
}


void publishSensorData() {
    // Create a formatted string with all sensor values
    String sensorMsg = String(sensorData.frontLidar) + "," +
                       String(sensorData.backLidar) + "," +
                       String(sensorData.leftLidar) + "," +
                       String(sensorData.rightLidar) + "," +
                       String(sensorData.frontLeftSonar) + "," +
                       String(sensorData.frontRightSonar) + "," +
                       String(sensorData.backLeftSonar) + "," +
                       String(sensorData.backRightSonar);
    
    mqttClient.beginMessage(sensorsTopicPublish);
    mqttClient.print(sensorMsg);
    mqttClient.endMessage();
}

// --------------------------------------------------------- MAIN -------------------------------------------------------------
void setup() {
  // Serial.begin(9600);
  Serial.begin(115200);
  Serial.println("initializing RPC");
  RPC.begin();
  delay(2000); 
  Serial.println("RPC initialized");

  pinMode(redLED, OUTPUT);//set red LED as output
  pinMode(grnLED, OUTPUT);//set green LED as output
  pinMode(ylwLED, OUTPUT);//set yellow LED as output

  setupMQTTConnection();
}
unsigned long lastSensorPublish = -5000;
void loop() {
  mqttClient.poll();
  updateSensorData();
  // Then do time-based publishing
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastSensorPublish >= sensorPublishInterval) {
    lastSensorPublish = currentMillis;
    publishSensorData();
    Serial.println("published sensor data");
  }
}

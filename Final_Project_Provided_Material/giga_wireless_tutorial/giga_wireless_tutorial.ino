/*
MQTT Simple Subscribe (GIGA R1 WiFi / Portenta H7)

This sketch demonstrates how to subscribe to a topic
from an MQTT broker.Data is NOT encrypted and this sketch is for
prototyping purposes only.

created 23 March 2023
by Karl Söderby

modified February 16, 2024
by Harman Singh
*/

#include <ArduinoMqttClient.h>
#include <WiFi.h>

char ssid[] = "RHIT-OPEN";        // your network SSID
char pass[] = "";    // your network password

//create the objects
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

//define broker, port and topic
// const char broker[] = "mosquitto.csse.rose-hulman.edu";
const char broker[] = "broker.hivemq.com";
int port = 1883;
const char topicSubscribe[]  = "ece445/parkj10/to_arduino";
const char topicPublish[]  = "ece445/parkj10/to_matlab";
unsigned long previousMillis = 0;
const long interval = 10000;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
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

  // topics can be unsubscribed using:
  // mqttClient.unsubscribe(topic);

  Serial.print("Topic: ");
  Serial.println(topicSubscribe);

  Serial.println();
}

void loop() {
  // call poll() regularly to allow the library to receive MQTT messages and
  // send MQTT keep alives which avoids being disconnected by the broker
  mqttClient.poll();

  unsigned long currentMillis = millis();
  //send a message every 10 seconds (based on interval)
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sendMessage("Hey from Arduino");
  }
}

void onMqttMessage(int messageSize) {
  
  String topicName = mqttClient.messageTopic();
  // use the Stream interface to print the contents
  String message = "";
  while (mqttClient.available()) {
    message += (char)mqttClient.read();
  }

  // we received a message, print out the topic and contents
  Serial.print("Received a message: '"); Serial.print(message); 
  Serial.print("' from topic: '"); Serial.print(topicName); Serial.print("', length: "); Serial.println(messageSize);

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




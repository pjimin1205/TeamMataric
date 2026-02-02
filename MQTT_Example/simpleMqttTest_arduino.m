clc; clear all; close all;

mqClient = mqttclient("tcp://broker.hivemq.com");
subscribe(mqClient, "ece445/your_username/to_arduino","Callback",@showSentMessage)
subscribe(mqClient, "ece445/your_username/to_matlab","Callback",@showReceivedMessage)

for k = 1:3
    message = sprintf('Hello from matlab %d',k);
    write(mqClient, "ece445/your_username/to_arduino", message)
    pause(2)
end

% unsubscribe(mqClient)

function showSentMessage(topic,data)
    string = strcat("Sent message: '", data, "' | to topic: '", topic, "'\n");
    fprintf(string)
end

function showReceivedMessage(topic,data)
    string = strcat("received message: '", data, "' | from topic: '", topic, "'\n");
    fprintf(string)
end

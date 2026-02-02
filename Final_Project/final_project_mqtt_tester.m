clc; clear all; close all;

% Setup MQTT connection
mqClient = mqttclient("tcp://broker.hivemq.com");
subscribe(mqClient, "ece445/parkj10/to_arduino", "Callback", @showSentMessage)
subscribe(mqClient, "ece445/parkj10/to_matlab", "Callback", @showReceivedMessage)

fprintf('=== Arduino MQTT Command Interface ===\n');
fprintf('Connected to broker and listening for Arduino messages...\n');
fprintf('Type your commands and press Enter to send.\n');
fprintf('Type "quit" or "exit" to stop.\n\n');

% Small delay to ensure subscriptions are active
pause(1);

% Main command loop
while true
    % Get user input
    userCommand = input('Enter command: ', 's');
    
    % Check for exit command
    if strcmpi(userCommand, 'quit') || strcmpi(userCommand, 'exit')
        fprintf('\nExiting...\n');
        break;
    end
    
    % Skip empty commands
    if isempty(userCommand)
        continue;
    end
    
    % Send command to Arduino
    write(mqClient, "ece445/parkj10/to_arduino", userCommand);
    
    % Small pause to allow message processing and response
    pause(0.2);
end

% Cleanup
unsubscribe(mqClient, "ece445/parkj10/to_arduino");
unsubscribe(mqClient, "ece445/parkj10/to_matlab");
clear mqClient;
fprintf('Disconnected.\n');

% Callback functions
function showSentMessage(topic, data)
    timestamp = datestr(now, 'HH:MM:SS');
    fprintf('[%s] >> SENT: %s\n', timestamp, data);
end

function showReceivedMessage(topic, data)
    timestamp = datestr(now, 'HH:MM:SS');
    fprintf('[%s] << ARDUINO: %s\n', timestamp, data);
end
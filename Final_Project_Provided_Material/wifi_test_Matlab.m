% Very complex code

client = tcpclient("192.164.123.19",23,"Timeout",5); % Change IP Address
write(client,'This is a test')

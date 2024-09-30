This project is made for my bachelor degree's thesis.

This project is called IoT Integrated Automatic Humidity Control System using Mamdani Fuzzy Logic. This project's goal is to make a small room miniature with the size of 30cm x 30cm x 30cm.
This room miniature will have its humidity maintained in the ideal humidity for a room, which is between 40%-60% Relative Humidity (RH). To control the humidity in this room miniature,
you will need to use a humidifier and a dehumimdifier. Both humidifier and dehummidifier will be controlled by a PWM module that controll their voltage depending on the humidity read by a sensor.

This project will be integrated by an IoT system that can allow you to see the current humidity of the room miniature anywhere you are as long as you have an access to the internet.
The humidity and temperature data will be shown to Node-Red UI through a local server. The humidity and temperature chart can be seen through smartphone or laptop. 
The data of humidity and temperature will also be stored in a database, made with Google Spreadsheets.

In this project, you will need these:
Arduino IDE (To code your microcontroller)
Mosquitto (This is an MQTT broker)
Node-Red (For MQTT User Interface)
Google SpreadSheet App Script (This is for storing your data from your sensor)



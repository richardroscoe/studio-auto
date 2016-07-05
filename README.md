Studio Automation Project
=========================

I have a purpose built studio in my garden and wanted to add some
automation to it. The aim was to allow lighting and heating to be
controlled remotely, via a web app.

There are 2 heaters in the studio, these are plugged into the main
circuit via remotely controlled sockets on 433MHz.

The first piece of equipment is a controller to transmit commands to
the sockets over the 433MHz radio frquency. This device also has a
temperature and humidity sensor onboard and an LCD display to allow
information to be displayed. This device employs an ESP-8266 
(this is a WiFi module that also has some general purpose I/O pins)

Lights are controlled via relays that are operated by another ESP-8266.
Some lights are on a dimmer circuit, these are controlled via the
Phase Angle method (and so needs to know when mains voltage crosses
0v).

Sensor information is published to/from an MQTT broker running on a
raspberry Pi.

The directories in this repository contain the following:

* www - The code for the web app
* arduino/studio-light-ctl - The code for the ESP-8266 controller
* arduino/studio-sensors-8266 - The code for the sensor hardware and 433MHz rf 


# RPM #

## Microcontroller Board ##
Arduino Mega 2560 R3

## Changelog ##

### v0.1 ###
* Only LCD showing message, and template for temperature display
* Add a counter that showed for how long since the last restart/initialization

### v0.2 ###
* Add code for One Wire DS1820 Temperature Probe.
* Temperature displayed on Serial Monitor

### v0.3 ###
* Temperature is now displayed on LCD
* Add oC on LCD

### v0.4 ###
* Add another display template, that only showed "Display 2" on the LCD
* Add a pushbutton algorithm that using 2 pushbutton to cycle between the display

### v0.5 ###
* "Display 2" is now changed to display measurement for pH
* Add another display template to show measurement for D.O and Salinity
* Add unit for each corresponding measurement

### v0.6 ###
* Cycling between display now only using one pushbutton
* Update the pushbutton algorithm for cycling display using only one pushbutton
* LCD pin 15 now controlled by Arduino digital pin to control the LCD backlight via PWM

### v0.7 ###
* Add a pushbutton mechanism that toggle the LCD after pressing the button for 1.5 second

### v0.8 ###
* Add another code for enabling data transmission through wireless RF Antenna
* Changing the pin used by DS18B20 from pin 8 to pin 9
* Changing the pin 11 used by LCD to pin 8 in prior to pin 11 used by vrtualwire library and would cause error in LCD display
* Now available to send temperature measurement + dummy data to another MCU that equipped with a wireless RF receiver

### v0.9 ###
* Add another code for pH measurement using dfRobot pH meter
* Displaying pH mesaurement result to LCD
* Data transmition happen for every 5 second with 5 packet of data sent at once

### v0.10 ###
* pH measurement code now placed in a specific function called getpH

### v0.11 ###
* Transmitting temperature and ph
* using dtostrf to convert float to string

### v0.12 ###
* if pH below 10, add a "0" in front of the value sent to HMI

### v0.14 ###
* dummy data added

### v0.15 ###
* DO added

### v0.16 ###
* DO temperature compensation every 10 measurement

### v0.17 ###
* Controller size update to 40
* digit compensation for every controller
* temp probe error code changed to -9 from -1000 to prevent controller overload when transmitting data
* LCD default is off

### v0.18 ###
* Turbidity measurement code now placed in a specific function called getTurbidity
* Smoothen the Turbidity measurement result by averaging and sorting 10 result in one measurement 

### v0.19 ###
* LCD cycling algorithm is revised and simplified      
* Depth is now measured every 2 second     

### v0.20 ###
* Turbidity read now measured in mV to increase sensitivity

### v0.21 ###
* Sensor reading is now done not simultaneously
* Every own sensor has its own reading window to prevent distorted result for another sensor


# Coop_Motor
Arduino code to automatically open and close a door based on calculating sunrise and sunset; specifically for a chicken coop.

# Hardware: 
  * Atmega328p (uses interrupts specific to that device)
      * Self built, doesn't have USB or voltage regulator
  * BTS7960B Motor Driver to operate the linear actuator
  * DS3232 Real Time Clock (RTC) to keep track of time and set alarms
      * https://datasheets.maximintegrated.com/en/ds/DS3232.pdf
  * 12v Linear actuator
  * 2 push buttons
  * LM2596S-ADJ DC-DC 3A Buck Converter (12v to 5v)
  * 12v 7amp-hour battery
  * 12v 7 watt Amorphous Solar Panel

# Libraries  
  * [Arduino DS3232RTC Library by Jack Christensen] (https://github.com/JChristensen/DS3232RTC)
  * Time and TimeAlarms by Michael Margolis
  
# Normal operation:
  
* There are two push buttons; one to open the door and another to close the door.  They are wired to different digital inputs but both go to the same interrupt on the board.
* The RTC keeps track of time and alarms that generate an interrupt.  
* Two alarms are used, ALARM_1 is set for sunrise plus offset to open the door and is set for sunset plus offset to close the door.
* ALARM_2 is set for 10 minutes after midnight to calculate and set the sunrise alarm for ALARM_1.

* When opening and closing the door, the motor will run until it reaches a defined timeout.  
* The sunrise and sunset alarms are calculated using the equations from this NOAA spreadsheet:
  * (http://www.esrl.noaa.gov/gmd/grad/solcalc/calcdetails.html)

# Cautions:

* There are no sensors to determine if someone/thing is in the way of the door and could cause harm/injury to what is blocking the door.  This could also cause the motor to burn out since it will run for a specified amount of time.
* By default the door is set to automatically close and there are not any sensors to detect if an animal is in front of door.
  
# Power Usage:

* In sleep mode: ~8mA
* While operating actuator: ~.51A
* Actuator: 3.5A to start

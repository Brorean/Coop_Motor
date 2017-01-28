#include <avr/sleep.h>
#include <DS3232RTC.h>
#include <Time.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <Wire.h>
#include <math.h>

/* 
 * Version: 3.0
 * Date: 1/27/2017
 * 
 * Hardware: 
 *    * Atmega328p (uses interrupts specific to that device)
 *      * Self built, doesn't not have USB or voltage regulator
 *    * BTS7960B Motor Driver to operate the linear actuator
 *    * DS3231 Real Time Clock (RTC) to keep track of time and set alarms
 *    * 12v Linear actuator
 *    * 2 push buttons
 *    * LM2596S-ADJ DC-DC 3A Buck Converter (12v to 5v)
 *    * 12v 7amp-hour battery
 *    * 12v 7 watt Amorphous Solar Panel
 * 
 * Normal operation:
 * 
 * There are two push buttons; one to open the door and another to 
 * close the door.  They are wired to different digital inputs but
 * both go to the same interrupt on the board.
 * The RTC keeps track of time and alarms that generate an interrupt.  
 * Two alarms are used, ALARM_1 is set for sunrise plus offset to open the door.
 * ALARM_2 is set for sunset plus offset to close the door.  Also when 
 * ALARM_2 is trigger it will calculate the next day's alarms.
 * The sunrise and sunset alarms are calculated using the NOAA spreadsheet:
 * (http://www.esrl.noaa.gov/gmd/grad/solcalc/calcdetails.html)
 *  Upon opening, the motor will run until it reaches a defined timeout.  
 *  When closing the motor will run for timeout*2 since the motor runs
 *  at 50% speed.
 * 
 * Cautions:
 *    * There are no sensors to determine if someone/thing is
 *    * in the way of the door and could cause harm/injury
 *    * to what is blocking the door.  This could also cause
 *    * the motor to burn out since it will run for a specified
 *    * amount of time.
 *    * By default the door is set to automatically close and there are not 
 *    * any sensors to detect if an animal is in front of door.
 *    * When closing the door the motor runs at 50% speed
 *   
 *   Power Usuage:
 *    * In sleep mode: ~8mA
 *    * While operating actuator: ~.51A
 *    * Actuator: 3.5A to start
 */
 
 /* Turn on/off autoclose */
const boolean autoClose = true;

 /* Define pins for interrupts */
const uint8_t wakePin = 2; //pin used for waking up
const uint8_t buttonPin = 3; //pin used for waking up from button push
const uint8_t alarmInt = 0;  //interrupt # for alarm
const uint8_t buttonInt = 1; //interrupt # for push buttons
boolean       alarmIntFlag = false;  //Flag for alarm interrupt
boolean       buttonIntFlag = false; //Flag for button interrupt
boolean       globalIsSunset = false; //Flag for motor close

/* Define pins for push buttons and motor */
const uint8_t openButtonPin = 7; //button input to open door
const uint8_t closeButtonPin = 8; //button input to close door
const uint8_t RPWMPin = 10; //Motor R-PWM
const uint8_t LPWMPin = 11; //Motor L-PWM
const uint8_t enablePin = 12; //Motor R/L Enable

/* Define variables for motor movement */
const uint16_t motorTimeout = 25000; //Run motor for this amount of time
const uint16_t motorCloseTimeout = motorTimeout * 2;
uint32_t       startTime = 0;  //Is set when automatically opening door
boolean        motorMoving = false; //Flag for motor moving

/* Variables to mathematically determine sunrise/sunset */
const float latitude = 38.89511 // (+ to N)
const float longitude = -77.03637; // (- to E)
const float timezone = -5.0;
const float julianNoon = 0.5;
const float sunriseOffset = 30.0; //minutes to add to sunrise
const float sunsetOffset  = 20.0; //minutes to add to sunset
const uint8_t minutesInHour = 60;

/* Function for the alarm interrupt */
void wakeUpNow() {
  sleep_disable();
  detachInterrupt(alarmInt);
  detachInterrupt(buttonInt);
  alarmIntFlag = true;
}

/* Function for the push button interrupt */
void wakeUpButton() {
  sleep_disable();
  detachInterrupt(buttonInt);
  detachInterrupt(alarmInt);
  buttonIntFlag = true;
}

void sleepNow() {
  //delay(1000);
  //set_sleep_mode(SLEEP_MODE_IDLE);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
  sei();
  attachInterrupt(alarmInt, wakeUpNow, LOW);
  attachInterrupt(buttonInt, wakeUpButton, RISING);
  sleep_cpu();

  //wakeup starts here
  sleep_disable();
}

void setup() {
  Serial.begin(115200);

  setupTime();
  
  pinMode(wakePin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(openButtonPin, INPUT);
  pinMode(closeButtonPin, INPUT);

  pinMode(RPWMPin, OUTPUT);
  pinMode(LPWMPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  motorStop();
 
  setupAlarms(false);
  
  Serial.println(F("RDY"));
  
}

void loop() {
  if (buttonIntFlag == true) {
    handleButtonPush();
  } else if (alarmIntFlag == true) {
    alarmIntFlag = false;
    setupTime();  //Re-sync time library with RTC
    globalIsSunset = handleAlarm();
  } else if (startTime != 0) { //Motor is moving from Alarm
    checkForMotorStop(globalIsSunset);
  } else {
    globalIsSunset = false;
    Serial.println(F("Sleeping"));
    motorStop(); //Just In Case but shouldn't be necessary
    Serial.flush();
    sleepNow();
  }
}

void setupAlarms(boolean isSunset) {
  time_t t_now = now(); //get current RTC time
  time_t t_temp = t_now;
  printDateTime(t_now);

  if (isSunset) {
    int hoursToMidnight = 24 - hour(t_now);
    t_temp = t_now + (hoursToMidnight * minutesInHour * 60);
    printDateTime(t_temp);
  }
  /* All the maths to calculate sunrise and sunset based on month/day */
  float JDN = getJulianDayFromUnix(t_temp);
  float JC = getJulianCenturyFromJulianDay(JDN);
  float geomMeanLongSun = getGeomMeanLongSunInDeg(JC);
  float geomMeanAnomSun = getGeomMeanAnomSunInDeg(JC);
  float eccentEarthOrbit = getEccentEarthOrbit(JC);
  float sunEqOfCtr = getSunEqOfCtr(JC, geomMeanAnomSun);
  float sunTrueLong = getSunTrueLongInDeg(geomMeanLongSun, sunEqOfCtr);
  //float sunTrueAnom = getSunTrueAnomInDeg(geomMeanAnomSun, sunEqOfCtr);
  //float sunRadVector = getSunRadVectorInAUs(eccentEarthOrbit, sunTrueAnom);
  float sunAppLong = getSunAppLongInDeg(sunTrueLong, JC);
  float meanObliqEcliptic = getMeanObliqEclipticInDeg(JC);
  float obliqCorr = getObliqCorrInDeg(meanObliqEcliptic, JC);
  //float sunRtAscen = getSunRtAscenInDeg(sunAppLong, obliqCorr);
  float sunDeclin = getSunDeclinInDeg(sunAppLong, obliqCorr);
  float varY = getVarY(obliqCorr);
  float eqOfTime = getEqOfTimeInMin(varY, eccentEarthOrbit, geomMeanLongSun, geomMeanAnomSun);
  float haSunrise = getHASunriseInDeg(sunDeclin);
  float solarNoon = getSolarNoon(eqOfTime);
  float sunrise = getSunrise(solarNoon, haSunrise);
  float sunset = getSunset(solarNoon, haSunrise);

  float minutesFromMidnightSunrise = getNumMinutesFromJulianFraction(sunrise) + sunriseOffset;
  uint8_t sunriseHour = minutesFromMidnightSunrise / minutesInHour;
  uint8_t  sunriseMinutes = (uint16_t)minutesFromMidnightSunrise % minutesInHour;

  float minutesFromMidnightSunset = getNumMinutesFromJulianFraction(sunset) + sunsetOffset;
  uint8_t sunsetHour = minutesFromMidnightSunset / minutesInHour;
  uint8_t  sunsetMinutes = (uint16_t)minutesFromMidnightSunset % minutesInHour;

  if (isSunset) {
    uint32_t waitTime = 1;
    if (sunsetHour > hour(t_now)) {
      waitTime = minutesFromMidnightSunset - (hour(t_now) * minutesInHour + minute(t_now));
      waitTime = ((waitTime + 1) * 60) * 1000; //add one minute, convert to milliseconds
    } else if (sunsetHour == hour(t_now)) {
      if (sunsetMinutes >= minute(t_now)) {
        waitTime = minutesFromMidnightSunset - (hour(t_now) * minutesInHour + minute(t_now));
        waitTime = ((waitTime + 1) * 60)* 1000; //add one minute, convert to milliseconds
      }
    }
    Serial.print(F("Going to delay setting sunset alarm: "));
    Serial.println(waitTime);
    Serial.flush();
    delay(waitTime);
  }
  
  RTC.setAlarm(ALM1_MATCH_HOURS, 0, sunriseHour, sunriseMinutes, 0);
  RTC.alarmInterrupt(ALARM_1, true);
  RTC.setAlarm(ALM2_MATCH_HOURS, 0, sunsetHour, sunsetMinutes, 0);
  RTC.alarmInterrupt(ALARM_2, true);
  
  Serial.print(F("Setting Sunrise alarm for "));
  Serial.print(sunriseHour);
  Serial.print(F(":"));
  Serial.println(sunriseMinutes);

  Serial.print(F("Setting Sunset alarm for "));
  Serial.print(sunsetHour);
  Serial.print(F(":"));
  Serial.println(sunsetMinutes);
}

void checkForMotorStop(boolean isSunset) {
  uint32_t currentTime = millis();
  uint16_t tempTimeout = 0;
  
  if (isSunset) {
    tempTimeout = motorCloseTimeout;
  } else {
    tempTimeout = motorTimeout;
  }
  
  if ((currentTime - startTime) > tempTimeout) {
    startTime = 0;
    motorStop();
  }
}

boolean handleAlarm() {
  
  if (RTC.alarm(ALARM_1)) {
    Serial.print(F("Alarm_1 Sunrise\t"));
    motorOpen();
    startTime = millis(); //Start timer to check for timeout
    if (startTime == 0) {
      /* Since the chip is put into power down mode there
       * is a small possiblity that the first millis()
       * could return 0
       * This is a problem because startTime of zero
       * is used to determine if the motor is not running
       * and in this case the motor should be running
       */
      startTime = 1;
    }
  } else if (RTC.alarm(ALARM_2)) {
    //Its sunset, close door generate new alarms
    Serial.println(F("Alarm_2 Sunset"));
    if (autoClose) {
      motorClose(false);
      setupAlarms(true);
      return true;
    }
  } else {
    Serial.println(F("Interrupt but no alarm"));
  }

  return false;
}

void handleButtonPush() {
  uint8_t openButtonStatus = digitalRead(openButtonPin);
  uint8_t closeButtonStatus = digitalRead(closeButtonPin);

  if ((openButtonStatus == HIGH) && (closeButtonStatus == HIGH)) {
    motorStop();
  } else if (openButtonStatus == HIGH) {
    motorOpen();
  } else if (closeButtonStatus == HIGH) {
    motorClose(true); 
  } else {
    motorStop();
    buttonIntFlag = false;
  }
}

void setupTime() {
  setSyncProvider(RTC.get);
  if (timeStatus() != timeSet) {
    Serial.println(F("Time set failed"));
  }  
}

void motorClose(boolean fullPower) {
  if (motorMoving == false) {
    Serial.println(F("Close Door"));
    digitalWrite(enablePin, HIGH);
    delay(1);
    if (fullPower) {
      digitalWrite(RPWMPin, LOW);
      digitalWrite(LPWMPin, HIGH);
    } else {
      digitalWrite(RPWMPin, LOW);
      analogWrite(LPWMPin, 127);
    }
    motorMoving = true;
  }
}

void motorOpen() {
  if (motorMoving == false) {
    Serial.println(F("Open Door"));
    digitalWrite(enablePin, HIGH);
    delay(1);
    digitalWrite(RPWMPin, HIGH);
    digitalWrite(LPWMPin, LOW);
    motorMoving = true;
  }
}

void motorStop() {
  Serial.println(F("Stop Motor"));
  digitalWrite(RPWMPin, LOW);
  digitalWrite(LPWMPin, LOW);
  digitalWrite(enablePin, LOW);
  motorMoving = false;
}

void printDateTime(time_t t) {
  printDate(t);
  Serial.print(" ");
  printTime(t);
  Serial.print("\n");
}

void printTime(time_t t) {
  printI00(hour(t), ':');
  printI00(minute(t), ':');
  printI00(second(t), ' ');
}

void printDate(time_t t) {
  printI00(day(t), 0);
  Serial.print(monthShortStr(month(t)));
  Serial.print(year(t), DEC);
}

void printI00(int val, char delim) {
  if (val < 10) {
    Serial.print(F("0"));
  }
  Serial.print(val, DEC);
  if (delim > 0) {
    Serial.print(delim);
  }
  return;
}

float getJulianDayFromUnix(uint32_t unixSecs) {
  //unixSecs is the date you want specified in seconds from 1/1/1970 GMT
  //  the time should be at midnight (0:0:0)
  // formula:
  //     ((unixSecs / 86400.0) + 2440587.5) to convert seconds from 1/1/1970 to julian
  //     add the julian time of noon because JDN starts at noon
  //     ((timezone/12) * julianNoon) is the ratio to convert timezone offset to julian
  return (((unixSecs / 86400.0) + 2440587.5) + julianNoon - ((timezone/12) * julianNoon));
   
}

float getJulianCenturyFromJulianDay(float julianDay) {
  return (julianDay - 2451545)/36525;
}

float getGeomMeanLongSunInDeg(float julianCentury) {
  return fmod(280.46646 + julianCentury * (36000.76983 + julianCentury * 0.0003032), 360.0);
}

float getGeomMeanAnomSunInDeg(float julianCentury) {
  return (357.52911 + julianCentury * (35999.05029 - 0.0001537 * julianCentury));
}

float getEccentEarthOrbit(float julianCentury) {
  return (0.016708634 - julianCentury * (0.000042037 + 0.0000001267 * julianCentury));
}

float getSunEqOfCtr(float julianCentury, float geomMeanAnomSun) {
  return (sin(degreesToRadians(geomMeanAnomSun)) * 
          (1.914602 - julianCentury * (0.004817 + 0.000014 * julianCentury)) + 
          sin(degreesToRadians(2 * geomMeanAnomSun)) * 
          (0.019993 - 0.000101 * julianCentury) + 
          sin(degreesToRadians(3 * geomMeanAnomSun)) * 0.000289);
}

float getSunTrueLongInDeg(float geomMeanLongSun, float sunEqOfCtr) {
  return (geomMeanLongSun + sunEqOfCtr);
}

/*
float getSunTrueAnomInDeg(float geomMeanAnomSun, float sunEqOfCtr) {
  return (geomMeanAnomSun + sunEqOfCtr);
}


float getSunRadVectorInAUs(float eccentEarthOrbit, float sunTrueAnom) {
  return ((1.000001018 * (1 - eccentEarthOrbit * eccentEarthOrbit)) / 
          (1 + eccentEarthOrbit * cos(degreesToRadians(sunTrueAnom))));
}
*/

float getSunAppLongInDeg(float sunTrueLong, float julianCentury) {
  return (sunTrueLong - 0.00569 - 0.00478 * sin(degreesToRadians(125.04 - 1934.136 * julianCentury)));
}

float getMeanObliqEclipticInDeg(float julianCentury) {
  return (23 + (26 + ((21.448 - julianCentury * 
          (46.815 + julianCentury * (0.00059 - julianCentury * 0.001813))))/60)/60);
}

float getObliqCorrInDeg(float meanObliqEcliptic, float julianCentury) {
  return (meanObliqEcliptic + 0.00256 * cos(degreesToRadians(125.04 - 1934.136 * julianCentury)));
}

/*
float getSunRtAscenInDeg(float sunAppLong, float obliqCorr) {
  return (radiansToDegrees(atan2(cos(degreesToRadians(obliqCorr)) * sin(degreesToRadians(sunAppLong)), 
                                 cos(degreesToRadians(sunAppLong)))));
}
*/

float getSunDeclinInDeg(float sunAppLong, float obliqCorr) {
  return (radiansToDegrees(asin(sin(degreesToRadians(obliqCorr)) * sin(degreesToRadians(sunAppLong)))));
}

float getVarY(float obliqCorr) {
  return (tan(degreesToRadians(obliqCorr/2)) * tan(degreesToRadians(obliqCorr/2)));
}

float getEqOfTimeInMin(float varY, float eccentEarthOrbit, float geomMeanLongSun, float geomMeanAnomSun) {
  return (4 * radiansToDegrees(varY * sin(2 * degreesToRadians(geomMeanLongSun))
          - 2 * eccentEarthOrbit * sin(degreesToRadians(geomMeanAnomSun)) 
          + 4 * eccentEarthOrbit * varY * sin(degreesToRadians(geomMeanAnomSun)) 
          * cos(2 * degreesToRadians(geomMeanLongSun)) - 0.5 * varY * varY 
          * sin(4 * degreesToRadians(geomMeanLongSun)) - 1.25 * eccentEarthOrbit * eccentEarthOrbit 
          * sin(2 * degreesToRadians(geomMeanAnomSun))));
} 

float getHASunriseInDeg(float sunDeclin) {
  return (radiansToDegrees(acos(cos(degreesToRadians(90.833)) /
          (cos(degreesToRadians(latitude)) * cos(degreesToRadians(sunDeclin))) - 
           tan(degreesToRadians(latitude)) * tan(degreesToRadians(sunDeclin)))));
}

float getSolarNoon(float eqOfTime) {
  return (720 - 4 * longitude - eqOfTime + timezone * 60) / 1440;
}

float getSunrise(float solarNoon, float haSunrise) {
  return ((solarNoon * 1440 - haSunrise * 4) / 1440);
}

float getSunset(float solarNoon, float haSunrise) {
  return ((solarNoon * 1440 + haSunrise * 4) / 1440);
}

float getSunlightDurationInMin(float haSunrise) {
  return (8 * haSunrise);
}

float getTrueSolarTime(float eqOfTime) {
  return fmod(julianNoon * 1440 + eqOfTime + 4 * longitude - 60 * timezone, 1440);
}

float getNumMinutesFromJulianFraction(float julianFraction) {
  return round((12.0 * (julianFraction/julianNoon)) * 60);
}

inline float degreesToRadians(float deg) {
  return (deg * M_PI / 180.0);
}

inline float radiansToDegrees(float rad) {
  return (rad * 180.0 / M_PI);
}

#include <avr/sleep.h>
#include <DS3232RTC.h>
#include <Time.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <Wire.h>
#include <math.h>
#include "Coop_Motor.h"

/* 
 * 
 * Hardware: 
 *    * Atmega328p (uses interrupts specific to that device)
 *      * Self built, doesn't have USB or voltage regulator
 *    * BTS7960B Motor Driver to operate the linear actuator
 *    * DS3232 Real Time Clock (RTC) to keep track of time and set alarms
 *    * 12v Linear actuator
 *    * 2 push buttons
 *    * LM2596S-ADJ DC-DC 3A Buck Converter (12v to 5v)
 *    * 12v 7amp-hour battery
 *    * 12v 7 watt Amorphous Solar Panel
 * 
 * Libraries:
 *     * Arudino DS3232RTC Library by Jack Christensen
 *     * Time and TimeAlarms by Michael Margolis
 *     
 * Normal operation:
 * 
 * There are two push buttons; one to open the door and another to 
 * close the door.  They are wired to different digital inputs but
 * both go to the same interrupt on the board.
 * The RTC keeps track of time and alarms that generate an interrupt.  
 * Two alarms are used, ALARM_1 is set for sunrise plus offset to open the door and
 * is set for sunset plus offset to close the door.  
 * ALARM_2 is set for 10 minutes after midnight to calculate and set the sunrise 
 * alarm for ALARM_1.
 * The sunrise and sunset alarms are calculated using the NOAA spreadsheet:
 * (http://www.esrl.noaa.gov/gmd/grad/solcalc/calcdetails.html)
 *  When opening and closing the door, the motor will run until it 
 *  reaches a defined timeout.  
 * 
 * Cautions:
 *    * There are no sensors to determine if someone/thing is
 *    * in the way of the door and could cause harm/injury
 *    * to what is blocking the door.  This could also cause
 *    * the motor to burn out since it will run for a specified
 *    * amount of time.
 *    * By default the door is set to automatically close and there are not 
 *    * any sensors to detect if an animal is in front of door.
 *   
 *   Power Usuage:
 *    * In sleep mode: ~8mA
 *    * While operating actuator: ~.51A
 *    * Actuator: 3.5A to start
 */
 
void setup() {
  Serial.begin(115200);

  init_rtc_time();
  
  pinMode(ALARM_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(OPEN_BUTTON_PIN, INPUT);
  pinMode(CLOSE_BUTTON_PIN, INPUT);

  pinMode(MOTOR_R_PWM_PIN, OUTPUT);
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  stop_door_movement();

  clear_rtc_status_register();
  setup_alarms(true, false);
  
  Serial.println(F("RDY"));
  
}

void loop() {
  if (g_button_int_flag == true)  {
    handle_button_push_interrupt();
  } 
  else if (g_alarm_int_flag == true) {
    g_alarm_int_flag = false;
    init_rtc_time();  //Re-sync time library with RTC
    handle_alarm_interrupt();
    clear_rtc_status_register();
  } 
  else if (g_motor_start_time != 0) { //Motor is moving from Alarm
    check_for_door_movement_timeout();
  } 
  else if (g_button_no_input_time != 0) {  //waiting for button timeout
    if (!(check_for_no_button_input_timeout())) {
      handle_button_push_interrupt();
    }
  }
  else {
    Serial.println(F("Sleeping"));
    stop_door_movement(); //Just In Case but shouldn't be necessary
    Serial.flush();
    put_to_sleep();
  }
}
/* Below functions are for handling interrupts
 *  and putting the microcontroller to sleep
 */
 
/* Function for the alarm interrupt */
void wake_up_from_alarm() {
  sleep_disable();
  detachInterrupt(ALARM_INTERRUPT);
  detachInterrupt(BUTTON_INTERRUPT);
  g_alarm_int_flag = true;
}

/* Function for the push button interrupt */
void wake_up_from_button_press() {
  sleep_disable();
  detachInterrupt(BUTTON_INTERRUPT);
  detachInterrupt(ALARM_INTERRUPT);
  g_button_int_flag = true;
}

void put_to_sleep() {
  //delay(1000);
  //set_sleep_mode(SLEEP_MODE_IDLE);  //used for debugging, don't want to shutdown all functions
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
  sei();
  attachInterrupt(ALARM_INTERRUPT, wake_up_from_alarm, LOW);
  attachInterrupt(BUTTON_INTERRUPT, wake_up_from_button_press, RISING);
  sleep_cpu();

  //wakeup starts here
  sleep_disable();
}

/* Below functions are related to setting alarms, handling alarms
 *  and RTC specific functions
 */
 
void setup_alarms(boolean is_init, boolean is_alarm_2) {
  uint8_t sunrise_hour = 0;
  uint8_t sunrise_minutes = 0;
  uint8_t sunset_hour = 0;
  uint8_t sunset_minutes = 0;

  get_sunrise_and_sunset(&sunrise_hour, &sunrise_minutes, &sunset_hour, &sunset_minutes);

  if (is_init) {
    //This is called from the setup() function
    if (is_sunrise_coming(sunrise_hour, sunrise_minutes)) {
      //the next interrupt is sunrise
      Serial.println(F("INIT, next ALARM_1 is sunrise"));
      g_is_sunrise = true;
      set_alarms_times(sunrise_hour, sunrise_minutes);
    } else {
      //the next interrupt is sunset
      Serial.println(F("INIT, next ALARM_1 is sunset"));
      g_is_sunrise = false;
      set_alarms_times(sunset_hour, sunset_minutes);
    }
  } else {
    if (is_alarm_2) {
      //the next interrupt is sunrise
      Serial.println(F("ALARM_2, setting ALARM_1 to sunrise"));
      g_is_sunrise = true;
      set_alarms_times(sunrise_hour, sunrise_minutes);
    } else {
      //the next interrupt is sunset
      Serial.println(F("ALARM_1, setting ALARM_1 to sunset"));
      g_is_sunrise = false;
      set_alarms_times(sunset_hour, sunset_minutes);
    }
  }
  
  Serial.print(F("Sunrise time: "));
  Serial.print(sunrise_hour);
  Serial.print(F(":"));
  Serial.println(sunrise_minutes);

  Serial.print(F("Sunset time: "));
  Serial.print(sunset_hour);
  Serial.print(F(":"));
  Serial.println(sunset_minutes);
}

void set_alarms_times(uint8_t alarm_hour, uint8_t alarm_minutes) {
  RTC.setAlarm(ALM1_MATCH_HOURS, 0, alarm_minutes, alarm_hour, 0);
  RTC.alarmInterrupt(ALARM_1, true);
  
  RTC.setAlarm(ALM2_MATCH_HOURS, 0, 0, 10, 0);
  RTC.alarmInterrupt(ALARM_2, true); 
}

void handle_alarm_interrupt() {  
  if (RTC.alarm(ALARM_1)) {
    Serial.print(F("Alarm_1\t"));
    if (g_is_sunrise) {
      setup_alarms(false, false); //set sunset alarm
      open_door();
    } else {
      if (AUTO_CLOSE) {
        //ALARM_2 will re-set the sunrise alarm
        close_door();
      }
    }
    g_motor_start_time = millis(); //Start timer to check for timeout
    if (g_motor_start_time == 0) {
      /* Since the chip is put into power down mode there
       * is a small possiblity that the first millis()
       * could return 0
       * This is a problem because startTime of zero
       * is used to determine if the motor is not running
       * and in this case the motor should be running
       */
      g_motor_start_time = 1;
    }
  } else if (RTC.alarm(ALARM_2)) {
    //It is midnight, set alarm_1 to sunrise
    Serial.println(F("Alarm_2 Set Sunrise Time"));
    setup_alarms(false, true);
  } else {
    Serial.println(F("Interrupt but no alarm"));
  }
}

void get_sunrise_and_sunset(uint8_t *sunrise_hour, uint8_t *sunrise_minutes, uint8_t *sunset_hour, uint8_t *sunset_minutes) {
  time_t current_time = now(); //get current RTC time
  print_date_time(current_time);
  
  /* All the maths to calculate sunrise and sunset based on month/day */
  float jdn = get_julian_day_from_unix(current_time);
  float jc = get_julian_century_from_julian_day(jdn);
  float geom_mean_long_sun = get_geom_mean_long_sun_in_deg(jc);
  float geom_mean_anom_sun = get_geom_mean_anom_sun_in_deg(jc);
  float eccent_earth_orbit = get_eccent_earth_orbit(jc);
  float sun_eq_of_ctr = get_sun_eq_of_ctr(jc, geom_mean_anom_sun);
  float sun_true_long = get_sun_true_long_in_deg(geom_mean_long_sun, sun_eq_of_ctr);
  //float sun_true_anom = get_sun_true_anom_in_deg(geom_mean_anom_sun, sun_eq_of_ctr);
  //float sun_rad_vector = get_sun_rad_vector_in_AUs(eccent_earth_orbit, sun_true_anom);
  float sun_app_long = get_sun_app_long_in_deg(sun_true_long, jc);
  float mean_obliq_ecliptic = get_mean_obliq_ecliptic_in_deg(jc);
  float obliq_corr = get_obliq_corr_in_deg(mean_obliq_ecliptic, jc);
  //float sun_rt_ascen = get_sun_rt_ascen_in_deg(sun_app_long, obliq_corr);
  float sun_declin = get_sun_declin_in_deg(sun_app_long, obliq_corr);
  float var_y = get_var_y(obliq_corr);
  float eq_of_time = get_eq_of_time_in_min(var_y, eccent_earth_orbit, geom_mean_long_sun, geom_mean_anom_sun);
  float ha_sunrise = get_ha_sunrise_in_deg(sun_declin);
  float solar_noon = get_solar_noon(eq_of_time);
  float sunrise = get_sunrise(solar_noon, ha_sunrise);
  float sunset = get_sunset(solar_noon, ha_sunrise);

  float minutes_from_midnight_sunrise = get_num_minutes_from_julian_fraction(sunrise) + SUNRISE_OFFSET;
  *sunrise_hour = minutes_from_midnight_sunrise / MINUTES_IN_HOUR;
  *sunrise_minutes = (uint16_t)minutes_from_midnight_sunrise % MINUTES_IN_HOUR;

  float minutes_from_midnight_sunset = get_num_minutes_from_julian_fraction(sunset) + SUNSET_OFFSET;
  *sunset_hour = minutes_from_midnight_sunset / MINUTES_IN_HOUR;
  *sunset_minutes = (uint16_t)minutes_from_midnight_sunset % MINUTES_IN_HOUR;
}

boolean is_sunrise_coming(uint8_t sunrise_hour, uint8_t sunrise_minutes) {
  boolean is_sunrise = false;
  time_t current_time = now(); //get current RTC time
  print_date_time(current_time);

  int current_hour = hour(current_time);
  //find out if the current hour is less than sunrise hour...if so sunrise hasn't happened
  if (current_hour == sunrise_hour) {
    int current_minute = minute(current_time);
    if (current_minute < sunrise_minutes) {
      is_sunrise = true;
    } 
  } else if (current_hour < sunrise_hour) {
    is_sunrise = true;
  }
    
  return is_sunrise;  
}

//Alarm has happened and door is auto opening/closing...check for timeout
void check_for_door_movement_timeout() {
  uint32_t current_time = millis();
  
  if ((current_time - g_motor_start_time) > MOTOR_MOVING_TIMEOUT) {
    g_motor_start_time = 0;
    stop_door_movement();
  }
}

void inline clear_rtc_status_register() {
  /* Clear the Control Status Register
   *  This is apparently fixes a bug...needs to be called after
   *  alarm is triggerred and handled
   */
  RTC.writeRTC(RTC_STATUS, 0x00);
}

void init_rtc_time() {
  setSyncProvider(RTC.get);
  if (timeStatus() != timeSet) {
    Serial.println(F("Time set failed"));
  }  
}

/* used for debugging the RTC */
void print_rtc_sram_values(boolean print_all_sram) {
  uint8_t addr = 0;
  uint8_t values[SRAM_START_ADDR + SRAM_SIZE]; //could use malloc
  uint16_t range = 0;
  if (print_all_sram) {
    range = SRAM_START_ADDR + SRAM_SIZE;  
  } else {
    range = SRAM_START_ADDR - 1;
  }
  
  Serial.println("Addr: DEC, HEX, BIN");
  RTC.readRTC(addr, values, range);
  for (addr = 0; addr < range; addr++) {
    Serial.print(addr, HEX);
    Serial.print(": ");
    Serial.print(values[addr], DEC);
    Serial.print(", ");
    Serial.print(values[addr], HEX);
    Serial.print(", ");
    Serial.println(values[addr], BIN);
  }
}

/* Functions for handling button input
 *  
 */
boolean check_for_no_button_input_timeout() {
  boolean timeout = false;
  uint32_t current_time = millis();

  if ((current_time - g_button_no_input_time) > BUTTON_TIMEOUT) {
    Serial.println(F("Button timeout, go to sleep"));
    g_button_no_input_time = 0;
    timeout = true;
  }
  
  return timeout;
}

void handle_button_push_interrupt() {
  uint8_t open_button_status = digitalRead(OPEN_BUTTON_PIN);
  uint8_t close_button_status = digitalRead(CLOSE_BUTTON_PIN);

  if ((open_button_status == HIGH) && (close_button_status == HIGH)) {
    //Both buttons are pushed...just stop
    g_button_no_input_time = millis();
    stop_door_movement();
  } else if (open_button_status == HIGH) {
    g_button_no_input_time = millis();
    open_door();
  } else if (close_button_status == HIGH) {
    g_button_no_input_time = millis();
    close_door(); 
  } else {
    //No buttons are pressed, stop motor
    //set button interrupt flag to false to put into sleep
    stop_door_movement();
    g_button_int_flag = false;
  }
}

/* Functions related to linear actuator */
void close_door() {
  if (g_is_motor_moving == false) {
    Serial.println(F("Close Door"));
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    delay(1);
    digitalWrite(MOTOR_R_PWM_PIN, LOW);
    digitalWrite(MOTOR_L_PWM_PIN, HIGH);
    g_is_motor_moving = true;
  }
}

void open_door() {
  if (g_is_motor_moving == false) {
    Serial.println(F("Open Door"));
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
    delay(1);
    digitalWrite(MOTOR_R_PWM_PIN, HIGH);
    digitalWrite(MOTOR_L_PWM_PIN, LOW);
    g_is_motor_moving = true;
  }
}

void stop_door_movement() {
  Serial.println(F("Stop Motor"));
  digitalWrite(MOTOR_R_PWM_PIN, LOW);
  digitalWrite(MOTOR_L_PWM_PIN, LOW);
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  g_is_motor_moving = false;
}

/* Functions for writing date and time to serial
 *  
 */
void print_date_time(time_t t) {
  print_date(t);
  Serial.print(" ");
  print_time(t);
  Serial.print("\n");
}

void print_time(time_t t) {
  print_I00(hour(t), ':');
  print_I00(minute(t), ':');
  print_I00(second(t), ' ');
}

void print_date(time_t t) {
  print_I00(day(t), 0);
  Serial.print(monthShortStr(month(t)));
  Serial.print(year(t), DEC);
}

void print_I00(int val, char delim) {
  if (val < 10) {
    Serial.print(F("0"));
  }
  Serial.print(val, DEC);
  if (delim > 0) {
    Serial.print(delim);
  }
  return;
}

/* Functions for calculating sunrise and sunset
 *  
 */
 
float get_julian_day_from_unix(uint32_t unix_secs) {
  //unixSecs is the date you want specified in seconds from 1/1/1970 GMT
  //  the time should be at midnight (0:0:0)
  // formula:
  //     ((unix_secs / 86400.0) + 2440587.5) to convert seconds from 1/1/1970 to julian
  //     add the julian time of noon because JDN starts at noon
  //     ((TIMEZONE/12) * JULIAN_NOON) is the ratio to convert timezone offset to julian
  return (((unix_secs / 86400.0) + 2440587.5) + JULIAN_NOON - ((TIMEZONE/12) * JULIAN_NOON));
   
}

float get_julian_century_from_julian_day(float julian_day) {
  return (julian_day - 2451545)/36525;
}

float get_geom_mean_long_sun_in_deg(float julian_century) {
  return fmod(280.46646 + julian_century * (36000.76983 + julian_century * 0.0003032), 360.0);
}

float get_geom_mean_anom_sun_in_deg(float julian_century) {
  return (357.52911 + julian_century * (35999.05029 - 0.0001537 * julian_century));
}

float get_eccent_earth_orbit(float julian_century) {
  return (0.016708634 - julian_century * (0.000042037 + 0.0000001267 * julian_century));
}

float get_sun_eq_of_ctr(float julian_century, float geom_mean_anom_sun) {
  return (sin(degrees_to_radians(geom_mean_anom_sun)) * 
          (1.914602 - julian_century * (0.004817 + 0.000014 * julian_century)) + 
          sin(degrees_to_radians(2 * geom_mean_anom_sun)) * 
          (0.019993 - 0.000101 * julian_century) + 
          sin(degrees_to_radians(3 * geom_mean_anom_sun)) * 0.000289);
}

float get_sun_true_long_in_deg(float geom_mean_long_sun, float sun_eq_of_ctr) {
  return (geom_mean_long_sun + sun_eq_of_ctr);
}

/*
float get_sun_true_anom_in_deg(float geom_mean_anom_sun, float sun_eq_of_ctr) {
  return (geom_mean_anom_sun + sun_eq_of_ctr);
}


float get_sun_rad_vector_in_AUs(float eccent_earth_orbit, float sun_true_anom) {
  return ((1.000001018 * (1 - eccent_earth_orbit * eccent_earth_orbit)) / 
          (1 + eccent_earth_orbit * cos(degrees_to_radians(sun_true_anom))));
}
*/

float get_sun_app_long_in_deg(float sun_true_long, float julian_century) {
  return (sun_true_long - 0.00569 - 0.00478 * sin(degrees_to_radians(125.04 - 1934.136 * julian_century)));
}

float get_mean_obliq_ecliptic_in_deg(float julian_century) {
  return (23 + (26 + ((21.448 - julian_century * 
          (46.815 + julian_century * (0.00059 - julian_century * 0.001813))))/60)/60);
}

float get_obliq_corr_in_deg(float mean_obliq_ecliptic, float julian_century) {
  return (mean_obliq_ecliptic + 0.00256 * cos(degrees_to_radians(125.04 - 1934.136 * julian_century)));
}

/*
float get_sun_rt_ascen_in_deg(float sun_app_long, float obliq_corr) {
  return (radians_to_degrees(atan2(cos(degrees_to_radians(obliq_corr)) * sin(degrees_to_radians(sun_app_long)), 
                                 cos(degrees_to_radians(sun_app_long)))));
}
*/

float get_sun_declin_in_deg(float sun_app_long, float obliq_corr) {
  return (radians_to_degrees(asin(sin(degrees_to_radians(obliq_corr)) * sin(degrees_to_radians(sun_app_long)))));
}

float get_var_y(float obliq_corr) {
  return (tan(degrees_to_radians(obliq_corr/2)) * tan(degrees_to_radians(obliq_corr/2)));
}

float get_eq_of_time_in_min(float var_y, float eccent_earth_orbit, float geom_mean_long_sun, float geom_mean_anom_sun) {
  return (4 * radians_to_degrees(var_y * sin(2 * degrees_to_radians(geom_mean_long_sun))
          - 2 * eccent_earth_orbit * sin(degrees_to_radians(geom_mean_anom_sun)) 
          + 4 * eccent_earth_orbit * var_y * sin(degrees_to_radians(geom_mean_anom_sun)) 
          * cos(2 * degrees_to_radians(geom_mean_long_sun)) - 0.5 * var_y * var_y 
          * sin(4 * degrees_to_radians(geom_mean_long_sun)) - 1.25 * eccent_earth_orbit * eccent_earth_orbit 
          * sin(2 * degrees_to_radians(geom_mean_anom_sun))));
} 

float get_ha_sunrise_in_deg(float sun_declin) {
  return (radians_to_degrees(acos(cos(degrees_to_radians(90.833)) /
          (cos(degrees_to_radians(LATITUDE)) * cos(degrees_to_radians(sun_declin))) - 
           tan(degrees_to_radians(LATITUDE)) * tan(degrees_to_radians(sun_declin)))));
}

float get_solar_noon(float eq_of_time) {
  return (720 - 4 * LONGITUDE - eq_of_time + TIMEZONE * 60) / 1440;
}

float get_sunrise(float solar_noon, float ha_sunrise) {
  return ((solar_noon * 1440 - ha_sunrise * 4) / 1440);
}

float get_sunset(float solar_noon, float ha_sunrise) {
  return ((solar_noon * 1440 + ha_sunrise * 4) / 1440);
}

float get_sunlight_duration_in_min(float ha_sunrise) {
  return (8 * ha_sunrise);
}

float get_true_solar_time(float eq_of_time) {
  return fmod(JULIAN_NOON * 1440 + eq_of_time + 4 * LONGITUDE - 60 * TIMEZONE, 1440);
}

float get_num_minutes_from_julian_fraction(float julian_fraction) {
  return round((12.0 * (julian_fraction/JULIAN_NOON)) * 60);
}


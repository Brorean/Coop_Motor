/* Turn on/off auto close */
const boolean AUTO_CLOSE = true;

 /* Define pins for interrupts */
const uint8_t ALARM_PIN   = 2; //pin used for waking up
const uint8_t BUTTON_PIN = 3; //pin used for waking up from button push
const uint8_t ALARM_INTERRUPT  = 0;  //interrupt # for alarm
const uint8_t BUTTON_INTERRUPT = 1; //interrupt # for push buttons
boolean       g_alarm_int_flag  = false;  //Flag for alarm interrupt
boolean       g_button_int_flag = false; //Flag for button interrupt

/* Define pins for push buttons and motor */
const uint8_t OPEN_BUTTON_PIN  = 7; //button input to open door
const uint8_t CLOSE_BUTTON_PIN = 8; //button input to close door
const uint8_t MOTOR_R_PWM_PIN  = 10; //Motor R-PWM
const uint8_t MOTOR_L_PWM_PIN  = 11; //Motor L-PWM
const uint8_t MOTOR_ENABLE_PIN = 12; //Motor R/L Enable

/* Define Variables for button timeouts */
const uint16_t BUTTON_TIMEOUT = 6000;  //How long to wait before putting controller to sleep
uint32_t       g_button_no_input_time = 0;  //keeps track of how long no button input

/* Define variables for motor movement */
const uint16_t MOTOR_MOVING_TIMEOUT = 25000; //Run motor for this amount of time ** WARNING MAX 65535
uint32_t       g_motor_start_time = 0;  //Is set when automatically opening door
boolean        g_is_motor_moving = false; //Flag for motor moving
boolean        g_is_sunrise = false;  //Flag for sunrise..ie open door if true...close if false

/* Variables to mathematically determine sunrise/sunset */
const float LATITUDE = 38.89511; // (+ to N)
const float LONGITUDE = -77.03637; // (- to E)
const uint8_t ALARM_2_HOURS = 0;  // Set the ALARM_2 hours field
const uint8_t ALARM_2_MINUTES = 10; // Set the ALARM_2 minutes field
const float TIMEZONE = -6.0;
const float SUNRISE_OFFSET = 30.0; //minutes to add to sunrise
const float SUNSET_OFFSET  = 20.0; //minutes to add to sunset
const float JULIAN_NOON = 0.5;
const uint8_t MINUTES_IN_HOUR = 60;

/* Function Declarations */
/* Interrupt functions */
void wake_up_from_button_press();
void wake_up_from_alarm();
void put_to_sleep();

/* RTC related functions */
void setup_alarms(boolean is_init, boolean is_alarm_2);
void set_alarms_times(uint8_t alarm_hour, uint8_t alarm_minutes);
void handle_alarm_interrupt();
void get_sunrise_and_sunset(uint8_t *sunrise_hour, uint8_t *sunrise_minutes, uint8_t *sunset_hour, uint8_t *sunset_minutes);
boolean is_sunrise_coming(uint8_t sunrise_hour, uint8_t sunrise_minutes);
void check_for_door_movement_timeout();
void clear_rtc_status_register();
void init_rtc_time();
void print_rtc_sram_values(boolean print_all_sram);

/* Button related functions */
boolean check_for_no_button_input_timeout();
void handle_button_push_interrupt();

/* Functions related to linear actuator */
void close_door();
void open_door();
void stop_door_movement();

/* Functions for writing date and time to serial */
void print_date_time(time_t t);
void print_time(time_t t);
void print_date(time_t t);
void print_I00(int val, char delim);

/* Functions for calculating sunrise and sunset */
float get_julian_day_from_unix(uint32_t unix_secs);
float get_julian_century_from_julian_day(float julian_day);
float get_geom_mean_long_sun_in_deg(float julian_century);
float get_geom_mean_anom_sun_in_deg(float julian_century);
float get_eccent_earth_orbit(float julian_century);
float get_sun_eq_of_ctr(float julian_century, float geom_mean_anom_sun);
float get_sun_true_long_in_deg(float geom_mean_long_sun, float sun_eq_of_ctr);
float get_sun_app_long_in_deg(float sun_true_long, float julian_century);
float get_mean_obliq_ecliptic_in_deg(float julian_century);
float get_obliq_corr_in_deg(float mean_obliq_ecliptic, float julian_century);
float get_sun_declin_in_deg(float sun_app_long, float obliq_corr);
float get_var_y(float obliq_corr);
float get_eq_of_time_in_min(float var_y, float eccent_earth_orbit, float geom_mean_long_sun, float geom_mean_anom_sun);
float get_ha_sunrise_in_deg(float sun_declin);
float get_solar_noon(float eq_of_time);
float get_sunrise(float solar_noon, float ha_sunrise);
float get_sunset(float solar_noon, float ha_sunrise);
float get_sunlight_duration_in_min(float ha_sunrise);
float get_true_solar_time(float eq_of_time);
float get_num_minutes_from_julian_fraction(float julian_fraction);

inline float degrees_to_radians(float deg) {
  return (deg * M_PI / 180.0);
}

inline float radians_to_degrees(float rad) {
  return (rad * 180.0 / M_PI);
}

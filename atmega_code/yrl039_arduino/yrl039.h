#ifndef YRL039_H
#define YRL039_H

#include <Wire.h>
#include <Math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Arduino.h"
#include "comms.h"
#include "piezo.h"

const boolean debug = true;
const boolean logSerialData = false;
const boolean use_serial = false;
const boolean use_i2c = true;

extern int stored_adc_value [6];

const int i2c_slave_address = 0x39;
const int power_good_pi = 2;
const int power_good_aux = 3;
const int enable_vr_pi = 4;
const int enable_vr_aux = 5;
const int power_led = 6;
const int power_button = 7;
const int i_sense_pi = A0;
const int i_sense_aux = A1;
const int v_batt = A2;
const int v_pi = A3;
const int v_aux = A6;
const int pcb_temp = A7;
const int fan_pwm = 9;
const int pi_shutdown_input = 10;
const int buzzer = 11; 
const int green_led = 12;
const int red_led = 13;    

const int battery_critical_voltage = 494;  //Below this voltage [approx 8.5V], hard-shutdown V.reg.
//const float pcb_fan_on_temperature = 25.0; //Fan will turn on at this temperature
//const float pcb_fan_temperature_range = 15.0; //Fan will be at 100% at this + fan_on_temperature
const int fan_on_threshold = 540; //Approximately 25-degree turn on
const int POWER_OFF = 1;
const int POWER_ON = 2;
const int BATTERY_FLAT = 10;

void printout_values(void);
float get_battery_voltage(void);
float get_pi_voltage(void);
float get_aux_voltage(void);
float get_pi_current(void);
float get_aux_current(void);
float get_temperature(void);

void set_fan_speed(float);
void set_int_speed(int);
void enable_vregs(void);
void disable_vregs(void);
void check_for_shutdown(void);
boolean check_battery(void);
void yrl039_setup(void);
void yrl039_loop(void);

#endif

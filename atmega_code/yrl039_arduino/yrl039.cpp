// Arduino Code for YRL039 - Power Supply Board for York Robotics Kit
//
// Copyright (C) 2019 James Hilder, York Robotics Laboratory
//
// This is free software, with no warranty, and you are welcome to redistribute it
// under certain conditions.  See GNU GPL v3.0.
//
// The YRL039 contains an Arduino compatible microcontroller  [ATmega328P] running at 8MHz [on a 3.3V supply]
//
// Choose Board = "Arduino Pro or Pro Mini" and Processor = "ATmega328P (3.3V, 8MHz)" in Tools Menu
// of Arduino IDE, or use settings in command-line script.
//
// In this version of the code, the Arduino controls the following hardware functions:
//   1.  Soft power control for the two 3A rated 5V supplies [R.Pi 5V & Aux 5V].
//   2.  Voltage & current monitoring of both 5V supplies, voltage monitoring of Vbatt
//   3.  Fan controller (with PCB temperature sensor)
//   4.  Piezo Sounder

#include "yrl039.h"

unsigned long startMillis;

int power_state = POWER_OFF;
int forced_shutdown_count = 0;
float fan_speed = 0;
boolean power_button_held = false;
boolean log_updated = false;
boolean fan_speed_set = false;
boolean fan_speed_override = false;
boolean force_shutdown = false;
boolean forced_shutdown_checked = false;
int stored_adc_value [6];
int adc_cycle = 0;

void check_for_shutdown() {
  //Pin pi_shutdown_input will be HIGH unless shutdown has been triggered on Pi
  if (force_shutdown || digitalRead(pi_shutdown_input) == LOW) {
    if (use_serial)Serial.println(F("Shutdown pin LOW: Switching off regulator"));
    chime(false);
    disable_vregs();
    delay(2000);
    power_state = POWER_OFF;
    force_shutdown = false;
  }
}

void setPwmFrequency(int divisor) {
  byte mode;
  switch (divisor) {
    case 1: mode = 0x01; break;
    case 8: mode = 0x02; break;
    case 64: mode = 0x03; break;
    case 256: mode = 0x04; break;
    case 1024: mode = 0x05; break;
    default: return;
  }
  TCCR1B = TCCR1B & 0b11111000 | mode;
}

void enable_vregs() {
  digitalWrite(enable_vr_aux, HIGH);
  delay(10);
  digitalWrite(enable_vr_pi, HIGH);
}

void disable_vregs() {
  digitalWrite(enable_vr_pi, LOW);
  delay(2);
  digitalWrite(enable_vr_aux, LOW);
  fan_speed_override = false;
}

/**
   Compare battery voltage to battery_critical reference.  If it is below, do a quick chime, force main V.reg off, enter an endless
   loop blocking booting [in order to boot, the Arduino will need to be reset, but this would generally happen anyway as the battery
   would be changed].  This state shouldn't really be reached as the R-Pi has its own battery monitoring and should shutdown at an
   earlier point.  A reference voltage of around 8.5V is probably a safe choice for use with 3-cell Li-Po batteries.
*/
boolean check_battery() {
  if (stored_adc_value[0] < battery_critical_voltage) {
    //Wait 10mS and check again
    if (use_serial){
      Serial.print(F("Battery voltage reports critically low:"));
      Serial.println(get_battery_voltage());
    }
    delay(10);
    stored_adc_value[0] = analogRead(v_batt);
    if (stored_adc_value[0] < battery_critical_voltage) {
      power_state = BATTERY_FLAT;
      if (use_serial)Serial.println(F("Switching off regulator"));
      alert();
      disable_vregs();
      return false;
    }
  }
  return true;
}

/**
   Sets fan to given float percentage (0=off, 1.0=max speed).  Uses offset of 35/255
*/
void set_fan_speed(float pct_speed) {
  float adj_speed = pct_speed * 220 + 35;
  int speed = (int) adj_speed;
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;
  analogWrite(fan_pwm, speed);
  fan_speed_set = true;
}


/**
   Sets fan to given int value (0=off, 220 = max speed)
*/
void set_int_speed(int int_speed) {
  int speed = int_speed + 35;
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;
  analogWrite(fan_pwm, speed);
  fan_speed_set = true;
}


void yrl039_setup() {
  // Setup serial console
  Serial.begin(115200);
  Serial.println(F("YRL039-Arduino Code"));
  analogReference(INTERNAL);  //Set ADC to 1.1V

  // Setup power supplies
  pinMode(enable_vr_pi, OUTPUT);
  pinMode(enable_vr_aux, OUTPUT);
  disable_vregs();

  // Setup fan PWM
  setPwmFrequency(1);
  pinMode(fan_pwm, OUTPUT);
  analogWrite(fan_pwm , 0);

  // Setup LEDs
  pinMode(power_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(pi_shutdown_input, INPUT_PULLUP);
  pinMode(power_button, INPUT_PULLUP);

  power_state = POWER_OFF;
  stored_adc_value[0] = analogRead(v_batt);
  if (check_battery())chime(true);

  if (use_i2c) {
    Wire.begin(i2c_slave_address);
    Wire.onRequest(i2c_request_event);
    Wire.onReceive(i2c_receive_event);
  }
  startMillis = millis();
}


void yrl039_loop() {
  unsigned long currentMillis = millis() - startMillis;
  if (currentMillis > 1024) currentMillis = currentMillis % 1024;  //Check this against optimsed code!
  switch(adc_cycle){
    case 0: stored_adc_value[0] = analogRead(v_batt);break;
    case 1: stored_adc_value[1] = analogRead(v_pi);break;
    case 2: stored_adc_value[2] = analogRead(v_aux);break;
    case 3: stored_adc_value[3] = analogRead(i_sense_pi);break;
    case 4: stored_adc_value[4] = analogRead(i_sense_aux);break;
    case 5: stored_adc_value[5] = analogRead(pcb_temp); break;
  }

  adc_cycle ++;
  if(adc_cycle > 5) adc_cycle = 0;
  
  //Check for serial data and print log info
  if (use_serial) {
    check_serial_for_data();
    process_serial_data();
    //If logSerialData flag is true in yrl039.h then print out a voltage\current status line over the serial bus every cycle [irrespective of operating state].  Serial bus only relevant when programming board [eg yrl030] is being used.
    if (logSerialData) {
      if (!log_updated) {
        if (currentMillis % 100 < 5) {
          printout_values();
        }
      } else {
        if (currentMillis % 100 > 5) {
          log_updated = false;
        }
      }
    }
  }

  //Check for I2C data
  if (use_i2c) process_i2c_data();

  switch (power_state) {
    case POWER_OFF:
      //In POWER_OFF state we have the VR disabled; check power button. Blink for 20ms, once a second
      if (digitalRead(red_led)) {
        if (currentMillis > 20) digitalWrite(red_led, 0);
      } else if (currentMillis < 21) digitalWrite(red_led, 1);
      if (digitalRead(power_button) == LOW) {
        Serial.println("Power button pressed");
        delay(20);
        if (check_battery()) {
          power_state = POWER_ON;
          enable_vregs();
          set_fan_speed(1);  // Turn fan on at max output on switch-on
          chime(true);
        }
      }
      break;

    case POWER_ON:
      //Check for forced shutdown
      if (currentMillis < 900) {
        if (!forced_shutdown_checked) {
          power_button_held = !(digitalRead(power_button));
          forced_shutdown_checked = true;
        }
      } else {
        if (forced_shutdown_checked) {
          if (power_button_held)forced_shutdown_count++;
          else forced_shutdown_count = 0;
          if (forced_shutdown_count == 4) force_shutdown = true;
          forced_shutdown_checked = false;
        }
      }
      //Set fan speed (once a second)
      if (!fan_speed_override) {
        if (currentMillis < 20) {
          if (!fan_speed_set) {
            //Original method - read temperature and use FP arithmetic
            //float target_speed = (get_temperature() - pcb_fan_on_temperature) / pcb_fan_temperature_range;
            //set_fan_speed(target_speed);  //Function has bounds checking+flag setting

            //Revised method using ints and stored value. 
            //Target speed wants to be zero at (and below) ~ 25C
            //and maximum speed at 40C
            //25C = 580mV [540 adc value]
            //40C = 674mV [628 adc value]
            //So approximate stored_adc_value - threshold[540] * 3
            int target_speed = (stored_adc_value[5] - fan_on_threshold) * 3;
            set_int_speed(target_speed);
          }
        } else fan_speed_set = false;
      }
      //Show heartbeat on RED led (2x20mS long flashes each ~250ms, then off for half a second)
      if (currentMillis < 512) {
        if (digitalRead(red_led)) {
          if (currentMillis % 256 > 20) digitalWrite(red_led, 0);
        } else if (currentMillis % 256 < 21) digitalWrite(red_led, 1);
      }
      check_battery();
      check_for_shutdown();
      break;
    case BATTERY_FLAT:
      if (currentMillis < 768) {
        if (digitalRead(red_led)) {
          if (currentMillis % 256 > 15) digitalWrite(red_led, 0);
        } else if (currentMillis % 256 < 16) digitalWrite(red_led, 1);
      }
      break;
  }
}

/**
  Return the battery voltage.  10 bit ADC, 1.1V = 1023.  [12V = 698] Bat_ref uses 1500K:100K PD:
*/
float get_battery_voltage() {
  return stored_adc_value[0] * 0.0172f;
}

/**
   Return the Pi voltage.  10 bit ADC, 1.1V = 1023.  [5V=927] 5V_ref uses 402K:100K PD:
*/
float get_pi_voltage() {
  return stored_adc_value[1] * 0.00539f;
}

/**
   Return the Aux voltage.  10 bit ADC, 1.1V = 1023. [5V=927] 5V_ref uses 402K:100K PD:
*/
float get_aux_voltage() {
  return stored_adc_value[2] * 0.00539f;
}

/**
   Return the Pi current.  10 bit ADC, 1.1V = 1023.  50x gain, 5mOhm res, 1A = 233.  Theoretical max = 4.3989A
*/
float get_pi_current() {
  return stored_adc_value[3] * 0.0043f;
}

/**
   Return the Aux current. 10 bit ADC, 1.1V = 1023.  50x gain, 5mOhm res, 1A = 233.  Theoretical max = 4.3989A
*/
float get_aux_current() {
  return stored_adc_value[4] * 0.0043f;
}

/**
   Return the temperature [TC1046].  0C=395. 6.25mV/C slope. 10 bit ADC, 1.1 = 1023.
*/
float get_temperature() {
  return (stored_adc_value[5] - 395) * 0.171875f;
}

void printout_values() {
  Serial.print(F("Vin:"));
  Serial.print(get_battery_voltage());
  Serial.print(F(" Vaux:"));
  Serial.print(get_aux_voltage());
  Serial.print(F(" Iaux:"));
  Serial.print(get_aux_current());
  Serial.print(F(" Vpi:"));
  Serial.print(get_pi_voltage());
  Serial.print(F(" Ipi:"));
  Serial.print(get_pi_current());
  Serial.print(F(" Temp:"));
  Serial.println(get_temperature());
  log_updated = true;
}

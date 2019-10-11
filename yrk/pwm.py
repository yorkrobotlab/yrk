#!/usr/bin/python
#
# York Robotics Kit Python API - Version 0.1
# Functions for the PCA9685PW PWM Driver [Servo Driver]
# Datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
# James Hilder, York Robotics Laboratory, Oct 2019

"""
.. module:: pwm
   :synopsis: Functions for the PWM [analog servo] driver

.. moduleauthor:: James Hilder <github.com/jah128>

This module provides functions for controlling the PCA9685 16-channel 12-bit
PWM driver, which is primarily intended for use with analog servo motors.

This has been quickly put together and the existing Adafruit library looks a
much more polished, elegant way of doing it, but requires circuitpython.

PCA9685 Datasheet
https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf

"""

import yrk.settings as s
import logging, os, smbus2, time

MODE1_REG = 0x00
MODE2_REG = 0x01
PRESCALE_REG = 0xFE

#Mode 1 Reg:
#RESTART:EXTCLK:AI:SLEEP:SUB1:SUB2:SUB3:ALLCALL
#Leave as 0x00 [reset state]

#Mode 2 Reg:
#RES:RES:RES:INVRT:OCH:OUTDRV:OUTNE
pwm_bus = smbus2.SMBus(s.I2C_5V_BUS)

def set_pwm_frequency(freq):
    #Prescale value = round(25MHz/4096 X Freq) - 1
    psv = round(6103.516/freq) - 1
    if psv>0xFF: psv=0xFF
    if psv<3: psv=3
    logging.debug("Target PWM frequency: %d Hz Prescale Value: %d  Actual Frequency: %4.1f Hz" % (freq,psv,6103.516/(psv+1)))
    #Prescale can only be reset when SLEEP register is set to 1
    set_sleep_mode()
    pwm_bus.write_byte_data(s.PWM_ADDRESS, PRESCALE_REG, psv)
    set_normal_mode()

def set_duty_cycle(output,dutycycle_pct):
    #Each output has a 4-byte register to set delay time and duty-cycle
    #We will fix delay time to zero
    logging.debug("Request duty cycle %2.2f%% on output %d" % (dutycycle_pct,output))
    if output < 0 or output > 15 or dutycycle_pct < 0 or dutycycle_pct > 100:
        logging.error("PWM request outside valid range (%d,%f)" % (output,dutycycle_pct))
    else:
        register_address = 6 + (output * 4)
        off_time = round(dutycycle_pct * 40.96)
        if(off_time > 0): off_time = off_time - 1
        bytes=[0,0,int(off_time) % 256,int(off_time) >> 8]
        logging.debug("Setting PWM bytes on channel %d to:" % output)
        logging.debug(bytes)
        pwm_bus.write_i2c_block_data(s.PWM_ADDRESS, register_address, bytes)

#Set MODE1 register to 0x10 [sleep state]
def set_sleep_mode():
    logging.debug("Setting sleep mode on PCA9685 PWM driver")
    pwm_bus.write_byte_data(s.PWM_ADDRESS, MODE1_REG, 0x10)

#Set MODE1 register to 0x20 [default on state, auto-increment]
def set_normal_mode():
    logging.debug("Setting normal mode on PCA9685 PWM driver")
    pwm_bus.write_byte_data(s.PWM_ADDRESS, MODE1_REG, 0x20)

#Test code
if __name__ == "__main__":
 s.init()
 logger = logging.getLogger()
 logger.setLevel(logging.DEBUG)
 logging.info("York Robotics Kit: Servo test code")
 #Sweep between 5% and 10% duty cycle at 50Hz [approx 1 - 2ms] on output 0
 set_pwm_frequency(50)
 phase=0
 while(True):
    for step in range(20):
            dc= (0.25 * step) + 5
            if phase == 1: dc = 15 - dc
            if(step == 19): phase = 1 - phase
            set_duty_cycle(0,dc)
            time.sleep(0.03)

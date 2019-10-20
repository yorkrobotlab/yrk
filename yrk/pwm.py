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

This module has been quickly put relatively quickly to provide functionality for
[exclusively for] use with analog servos.  If wanted for other purposes it would
be worth investigating the existing Adafruit library, which is more polished,
but requires circuitpython amongst other libraries and might take a bit of
adaptation to work using the switched i2c bus.

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
pwm_freq_hertz = 0;

#Calculate the nearest duty cycle value to target on-period in seconds, returns int [0-4095] for use with set_duty_cycle_raw()
def calculate_nearest_duty_cycle_to_period(period_seconds: float) -> int:
    """Calculate the value for raw cycle for a given period

    Args:
        period_seconds (float): The target on-time in seconds.

    Returns:
        int: The raw duty cycle value [*for use with set_duty_cycle_raw*]

    """

    if pwm_freq_hertz == 0: return 0
    pwm_period = 1.0 / pwm_freq_hertz
    if period_seconds <= 0: return 0
    if period_seconds >= pwm_period: return 4095
    return int( ( period_seconds * 4095 ) / pwm_period)

#Estimate the on-time based on the raw dutycycle value.  Return value in seconds
def estimate_on_time(dutycycle_raw: int) -> float:
    """Estimate the on-time based on the raw duty cycle value

    Args:
        dutycycle_raw (int): The target duty-cycle [*range 0 - 4095*]

    Returns:
        float: The approximate on-time value in seconds

    """

    return (dutycycle_raw / (4096 * pwm_freq_hertz))

def set_pwm_frequency(freq: float):
    """Sets the PWM frequency

    The PWM will be set as close as it can be to the requested frequency.  It
    calculates the closest ``prescale`` value and calls ``set_prescale_value``
    with this value.  All outputs have the same frequency.

    Args:
        freq (float): The target frequency in hertz [*effective range 24 - 1526*]

    """

    #Prescale value = round(25MHz/4096 X Freq) - 1
    psv = round(6103.516/freq) - 1
    set_prescale_value(psv)

def set_prescale_value(psv: int):
    """Sets the prescale register to set the PWM frequency

    PWM frequency approximately equal to ``6104 / (psv + 1)``

    Args:
        psv (int): The target prescale register value [*range 3-255*]

    """

    global pwm_freq_hertz
    if psv>0xFF:
        logging.warning("PWM prescale value out of range (%d), setting to 255" % psv)
        psv=0xFF
    if psv<3:
        logging.warning("PWM prescale value out of range (%d), setting to 3" % psv)
        psv=3
    pwm_freq_hertz = 6103.516/(psv+1)
    logging.debug("Prescale Value: %d  Actual Frequency: %4.1f Hz" % (psv,pwm_freq_hertz))
    #Prescale can only be reset when SLEEP register is set to 1
    set_sleep_mode()
    pwm_bus.write_byte_data(s.PWM_ADDRESS, PRESCALE_REG, psv)
    set_normal_mode()

def set_duty_cycle(output: int,dutycycle_pct: float):
    """Sets the duty cycle (on period) of a PWM output as a percentage

    Args:
        output (int):  The servo output to use [*range 0-15*]
        dutycycle_pct (float): The percentage [*0-100*] of on-time

    """

    #Each output has a 4-byte register to set delay time and duty-cycle
    #We will fix delay time to zero
    logging.debug("Request duty cycle %2.2f%% on output %d" % (dutycycle_pct,output))
    if output < 0 or output > 15 or dutycycle_pct < 0 or dutycycle_pct > 100:
        logging.error("PWM request outside valid range (%d,%f)" % (output,dutycycle_pct))
    else:
        off_time = round(dutycycle_pct * 40.96)
        if(off_time > 0): off_time = off_time - 1
        set_duty_cycle_raw(output,off_time)

def set_duty_cycle_raw(output: int,dutycycle_raw: int):
    """Sets the raw on-period value for a given PWM output

    Args:
        output (int):  The servo output to use [*range 0-15*]
        dutycycle_raw (int): The on-time period [*range 0-4095*]

    """

    if output < 0 or output > 15 or dutycycle_raw < 0 or dutycycle_raw > 4095:
        logging.error("PWM request outside valid range (%d,%d)" % (output,dutycycle_raw))
    register_address = 6 + (output * 4)
    bytes=[0,0,dutycycle_raw % 256,dutycycle_raw >> 8]
    logging.debug("Setting PWM bytes on channel %d to:" % output)
    logging.debug(bytes)
    pwm_bus.write_i2c_block_data(s.PWM_ADDRESS, register_address, bytes)

#Set MODE1 register to 0x10 [sleep state]
def set_sleep_mode():
    """Enables sleep mode on PWM driver"""

    logging.debug("Setting sleep mode on PCA9685 PWM driver")
    pwm_bus.write_byte_data(s.PWM_ADDRESS, MODE1_REG, 0x10)

#Set MODE1 register to 0x20 [default on state, auto-increment]
def set_normal_mode():
    """Disables sleep mode on PWM driver"""

    logging.debug("Setting normal mode on PCA9685 PWM driver")
    pwm_bus.write_byte_data(s.PWM_ADDRESS, MODE1_REG, 0x20)

#Test code
if __name__ == "__main__":
 s.init()
 logger = logging.getLogger()
 logger.setLevel(logging.DEBUG)
 logging.info("York Robotics Kit: Servo test code")
 set_pwm_frequency(50)

 #Code to sweep between 1.0ms and 2.0ms period [independant of pwm_freq] using calculate_nearest_duty_cycle_to_period and set_duty_cycle_raw
 phase=0
 while(True):
    for step in range(50):
        if(phase == 0): dc_period = 1000 + ( 20 * step )
        else: dc_period = 2000 - (20 * step)
        if step == 49: phase = 1 - phase
        raw_val = calculate_nearest_duty_cycle_to_period(0.000001 * dc_period)
        logging.info("Target period: %dms  Raw:%d  " % ( dc_period, raw_val ) )
        set_duty_cycle_raw(0,raw_val)
        time.sleep(0.04)


     # #Code to sweep between 5% and 10% duty cycle at 50Hz [approx 1 - 2ms] on output 0 using the set_duty_cycle(channel,percentage)
     # phase=0
     # while(True):
     #    for step in range(20):
     #            dc= (0.25 * step) + 5
     #            if phase == 1: dc = 15 - dc
     #            if(step == 19): phase = 1 - phase
     #            set_duty_cycle(0,dc)
     #            time.sleep(0.03)

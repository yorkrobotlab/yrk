#!/usr/bin/python
# York Robotics Kit Python API
#
# Version 0.1
# Functions for communicating with the YRL039 Power Supply Board
# James Hilder, York Robotics Laboratory, Oct 2019

"""
.. module:: power
   :synopsis: Functions for communicating with the YRL039 power supply PCB

.. moduleauthor:: James Hilder <github.com/jah128>

The YRL039 PCB provides the main power supplies for the YRK, using a pair of
3A, 5V buck [*step-down*] converters.  These power supplies are controlled by
a ATMega328 microcontroller, which also monitors the voltage of the supply plus
the voltages and currents of both 5V outputs.  The ATMega also monitors the
temperature of the PCB and controls a small fan, positioned directly above the
CPU of the Raspberry Pi.

This module provides the functions to read the data values from the ATMega
microcontroller [using the I2C bus].

"""

import logging, threading, time, random, os, smbus2                             #General Python imports
from smbus2 import i2c_msg
import yrk.settings as s

try:
  i2c_bus = smbus2.SMBus(s.YRL039_BUS)                                            #The YRL039 ATMega is attached to i2c_7
  init_okay = True
except FileNotFoundError:
  logging.error("[power.py]    : Cannot access /dev/i2c-%d"  % (s.YRL039_BUS))
  init_okay = False
  s.BUS_ERROR = True


v_batt=0                                                                        #Battery\DC in voltage
v_pi=0                                                                          #Raspberry Pi voltage [should be 5.0V]
v_aux=0                                                                         #Aux supply voltage [should be 5.0V]
i_pi=0                                                                          #Raspberry Pi current
i_aux=0                                                                         #Aux supply current
pcb_temp=0                                                                      #PCB Temp [sensor is next to buzzer]

# The YRL039 frequently reads voltages, current and temperature.  This can be read invidividually but it is recommended
# to use the read_all_values() function then the appropriate getter
def read_all_values():
    """Reads and stores all voltage, current and temperature readings in a single i2c request"""
    global v_batt, v_pi,v_aux,i_pi,i_aux,pcb_temp
    try:
      if init_okay:
        i2c_bus.write_byte(s.YRL039_ADDRESS,8)
        msg = i2c_msg.read(s.YRL039_ADDRESS,8)
        i2c_bus.i2c_rdwr(msg)
        vals=list(msg)
        v_batt = 0.0172 * ((vals[0] << 2) + (vals[1] >> 6));
        v_pi = 0.00539 * (((vals[1] & 0x3F) << 4) + (vals[2] >> 4));
        v_aux = 0.00539 * (((vals[2] & 0x0F) << 6) + (vals[3] >> 2));
        i_pi = 0.0043 * ((vals[4] << 2) + (vals[5] >> 6));
        i_aux = 0.0043 * (((vals[5] & 0x3F) << 4) + (vals[6] >> 4));
        pcb_temp = ((((vals[6] & 0x0F) << 6) + (vals[7] >> 2)) - 395) * 0.171875;
    except OSError:
      logging.error("Error reading register 8 from YRL039 (address %X bus %d)" % (s.YRL039_ADDRESS,s.YRL039_BUS))

def get_battery_voltage():
    """Returns battery voltage (float) stored on last call of ``read_all_values``"""
    return v_batt;

def get_pi_voltage():
    """Returns 5V_PI voltage (float) stored on last call of ``read_all_values``"""
    return v_pi;

def get_aux_voltage():
    """Returns 5V_AUX voltage (float) stored on last call of ``read_all_values``"""
    return v_aux;

def get_pi_current():
    """Returns 5V_PI current (float) stored on last call of ``read_all_values``"""
    return i_pi;

def get_aux_current():
    """Returns 5V_AUX current (float) stored on last call of ``read_all_values``"""
    return i_aux;

def get_pcb_temperature():
    """Returns PCB temperature (float) stored on last call of ``read_all_values``"""
    return pcb_temp;

def read_battery_voltage():
    return read_int_register(1) * 0.0172;

def read_pi_voltage():
    return read_int_register(2) * 0.00539;

def read_aux_voltage():
    return read_int_register(3) * 0.00539;

def read_pi_current():
    return read_int_register(4) * 0.0043;

def read_aux_current():
    return read_int_register(5) * 0.0043;

def read_pcb_temperature():
    return (read_int_register(6) - 395) * 0.171875;

def write_message(register,message):
    if init_okay: i2c_bus.write_i2c_block_data(s.YRL039_ADDRESS, register, message)


def read_int_register(register):
    try:
      if not init_okay:
          logging.warning("Error in initialisation of YRL039")
          return 0
      i2c_bus.write_byte(s.YRL039_ADDRESS,register)
      msg = i2c_msg.read(s.YRL039_ADDRESS,2)
      i2c_bus.i2c_rdwr(msg)
      vals=list(msg)
      return (vals[0] << 8) + vals[1];
    except OSError:
      logging.error("Error reading register from YRL039 (address %X bus %d)" % (s.YRL039_ADDRESS,s.YRL039_BUS))
      return 0

#Command line test prints out voltage, current and temperature readings [will run when yrl039.py is run directly]
if __name__ == "__main__":
 s.init()
 s.setup_logger("power")
 logging.info("York Robotics Kit: Power supply monitoring test code")

 #This method sends all 6 values as a single 8-byte I2C message
 read_all_values();
 logging.info("Vin: %.2f  Vpi: %.2f  Vaux: %.2f  Ipi: %.2f  Iaux: %.2f  PCB Temp: %.2f" % (v_batt,v_pi,v_aux,i_pi,i_aux,pcb_temp))

 #This method makes a 2 byte I2C request for each value
 v_batt = read_battery_voltage();
 v_pi = read_pi_voltage();
 v_aux = read_aux_voltage();
 i_pi = read_pi_current();
 i_aux = read_aux_current();
 pcb_temp = read_pcb_temperature();
 logging.info("Vin: %.2f  Vpi: %.2f  Vaux: %.2f  Ipi: %.2f  Iaux: %.2f  PCB Temp: %.2f" % (v_batt,v_pi,v_aux,i_pi,i_aux,pcb_temp))
 os._exit(1)

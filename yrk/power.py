#!/usr/bin/python
# York Robotics Kit Python API
#
# Version 0.1
#
# Functions for communicating with the YRL039 Power Supply Board
#
# James Hilder, York Robotics Laboratory, Oct 2019

import logging, threading, time, random, os, smbus2                             #General Python imports
from smbus2 import i2c_msg
import yrk.settings as s

i2c_bus = smbus2.SMBus(s.YRL039_BUS)                                            #The YRL039 ATMega is attached to i2c_7

v_batt=0                                                                        #Battery\DC in voltage
v_pi=0                                                                          #Raspberry Pi voltage [should be 5.0V]
v_aux=0                                                                         #Aux supply voltage [should be 5.0V]
i_pi=0                                                                          #Raspberry Pi current
i_aux=0                                                                         #Aux supply current
pcb_temp=0                                                                      #PCB Temp [sensor is next to buzzer]

# The YRL039 frequently reads voltages, current and temperature.  This can be read invidividually but it is recommended
# to use the read_all_values() function then the appropriate getter
def read_all_values():
    global v_batt, v_pi,v_aux,i_pi,i_aux,pcb_temp
    try:
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
    return v_batt;

def get_pi_voltage():
    return v_pi;

def get_aux_voltage():
    return v_aux;

def get_pi_current():
    return i_pi;

def get_aux_current():
    return i_aux;

def get_pcb_temperature():
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
    i2c_bus.write_i2c_block_data(s.YRL039_ADDRESS, register, message)


def read_int_register(register):
    try:
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
 logger = logging.getLogger()
 logger.setLevel(logging.DEBUG)

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

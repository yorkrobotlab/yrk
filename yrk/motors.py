#!/usr/bin/python
#
# York Robotics Kit Python API
#
# Version 0.1
#
# Functions for the i2c based motor drivers
#
# Datasheet: http://www.ti.com/lit/ds/symlink/drv8830.pdf
# James Hilder, York Robotics Laboratory, Oct 2019

import yrk.settings as s
import smbus2, logging
import time, sys, os

silent = False

i2c = smbus2.SMBus(s.YRL040_BUS)                                                #The motor drivers are attached to i2c_7

brake_states = [False,False,False,False]
motor_speeds = [0.0,0.0,0.0,0.0]

def get_brake_state(motor):
  return brake_states[motor]

def get_motor_speed(motor):
  return motor_speeds[motor]

#Check requested speed ranges from -1 to 1
def check_bounds(speed):
  if speed > 1.0: return 1.0
  if speed < -1.0: return -1.0
  if speed > -0.01 and speed < 0.01: return 0
  return speed

#Convert speed from [0.0-1.0] into adjusted integer
#We can set speed from range 6 (0.48V) to 63 (5.08V)
def get_integer_speed(speed):
  return int(abs ( speed * 56 ) + 6)

#Set motor speed to given, speed -1.0 to 1.0, zero will put in coast
def set_motor_speed(motor,speed):
  global brake_states,motor_speeds
  speed=check_bounds(speed)
  byte = 0
  brake_states[motor]=False
  motor_speeds[motor]=speed
  if speed == 0:
    if not silent:
        logging.info("Setting motor %d to coast" % motor)
  else:
    integer_speed = get_integer_speed(speed)
    if speed < 0:
        if not silent:
            logging.info("Setting motor {} speed to {} [{}V] backwards".format(motor,integer_speed,(0.08 * integer_speed)))
        byte = ( integer_speed << 2 ) + 1
    else:
        if not silent:
            logging.info("Setting motor {} speed to {} [{}V] forwards".format(motor,integer_speed,(0.08 * integer_speed)))
        byte = ( integer_speed << 2 ) + 2
  i2c.write_byte_data(s.MOTOR_ADDRESSES[motor], 0, byte)

#Set all motors to coast
def stop_all_motors():
    for motor in range(4):
        set_motor_speed(motor,0)

#Brake motor 1
def brake_motor(motor):
  global brake_states
  brake_states[motor]=True
  byte = 0x03  # IN1 & IN2 both high = brake
  if not silent:
    logging.info("Setting motor %d to brake" % motor)
  i2c.write_byte_data(s.MOTOR_ADDRESSES[motor], 0, byte)

#Test code
if __name__ == "__main__":
    s.init()
    logging.info("York Robotics Kit: Motor test code")
    #EG: Set motor 0 to 50% for 3 seconds
    set_motor_speed(0,0.5)
    time.sleep(3)
    stop_all_motors()
    os._exit(1)

# H-Bridge Logic
#
# Motors use DRV8830 (Left = 0x62, Right = 0x60)
# Datasheet: http://www.ti.com/lit/ds/symlink/drv8830.pdf
#
# IN1  IN2  OUT1  OUT2  FUNCTION
# 0    0    Z     Z     Standby/coast
# 0    1    L     H     Reverse
# 1    0    H     L     Forward
# 1    1    H     H     Brake
#
# Vset 5..0 control voltage (ie speed).  voltage = 0.08 x Vset
# Min value 0x06 (000110) = 0.48V
# Max value 0x3F (111111) = 5.06V
#
# Register 0 is control register, Register 1 is fault register
# Control register:
# Vset[5] Vset[4] Vset[3] Vset[2] Vset[1] Vset[0] In2 In1

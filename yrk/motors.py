#!/usr/bin/python
#
# York Robotics Kit Python API
# Version 0.1
# Functions for the i2c based motor drivers
# Datasheet: http://www.ti.com/lit/ds/symlink/drv8830.pdf
# James Hilder, York Robotics Laboratory, Oct 2019

"""
.. module:: motors
   :synopsis: Functions for H-Bridge motor drivers

.. moduleauthor:: James Hilder <github.com/jah128>

The YRL040 PCB contains 4 DRV8830 H-Bridge motor drivers, powered from the
5V_AUX supply.   Each driver has an 800mA current limit.  It is primarily
intended for use with small brushed DC motors such as the 12mm micro-metal
gear motors available from a number of suppliers including MFA Como, Pimoroni
and Pololu.  Motors are connected using the push-level Wago terminals at either
side of the YRL040 PCB.

The motor driver allows the effective motor voltage [in either direction] to be
programmed using an I2C message.  It also allows the motor to be put in a coast
[*high-impendance*] or brake [*short-circuit*] state.  The ICs contain fault
[*over-current* and *over-temperature*] conditions to trigger a logic low
output, which is connected the GPIO expander [see ``gpio``].

DRV8830 Data Sheet:
https://www.ti.com/lit/ds/symlink/drv8830.pdf
"""

import yrk.settings as s
import smbus2, logging
import time, sys, os

# logging_mode = logging.info           #Send all messages to info logging level
logging_mode = logging.debug          #Send all messages to debug logging level

try:
  i2c = smbus2.SMBus(s.YRL040_BUS)                                         #The motor drivers are attached to the YRL040 3.3V I2C Bus
  init_okay = True
except FileNotFoundError:
  logging.error("[motors.py]   : Cannot access /dev/i2c-%d"  % (s.YRL040_BUS))
  init_okay = False
  s.BUS_ERROR = True


brake_states = [False,False,False,False]
motor_speeds = [0.0,0.0,0.0,0.0]

def get_brake_state(motor):
    """Gets current brake state of the given motor driver

    Args:
        motor (int): The motor driver [*range 0-3*]

    Returns:
        bool: True if the motor is in brake [*short-circuit*] state
    """

    return brake_states[motor]

def get_motor_speed(motor):
    """Gets current speed of the given motor driver

    Args:
        motor (int): The motor driver [*range 0-3*]

    Returns:
        float: The target speed of the motor [*range -1.0 to 1.0*]
    """

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
  """Sets the motor to given speed

  Args:
    motor (int): The motor driver [*range 0-3*]
    speed (float): The target speed for the motor [*range -1.0 to 1.0*]

  """

  global brake_states,motor_speeds
  speed=check_bounds(speed)
  byte = 0
  brake_states[motor]=False
  motor_speeds[motor]=speed
  if speed == 0:
    logging_mode("Setting motor %d to coast" % motor)
  else:
    integer_speed = get_integer_speed(speed)
    if speed < 0:
        logging_mode("Setting motor {} speed to {} [{}V] backwards".format(motor,integer_speed,(0.08 * integer_speed)))
        byte = ( integer_speed << 2 ) + 1
    else:
        logging_mode("Setting motor {} speed to {} [{}V] forwards".format(motor,integer_speed,(0.08 * integer_speed)))
        byte = ( integer_speed << 2 ) + 2
  if init_okay: i2c.write_byte_data(s.MOTOR_ADDRESSES[motor], 0, byte)

#Set all motors to coast
def stop_all_motors():
    """Sets all 4 motor drivers to coast state"""
    for motor in range(4):
        set_motor_speed(motor,0)

#Brake motor 1
def brake_motor(motor):
  """Turns on brake mode for given motor

  Brake state effectively short-circuits the motor windings, resisting motion.
  This is different to coast state [*high-impendance*] which is set by calling
  ``set_motor_speed(motor,0)``

  Args:
    motor (int): The motor driver [*range 0-3*]

  """

  global brake_states
  brake_states[motor]=True
  byte = 0x03  # IN1 & IN2 both high = brake
  logging_mode("Setting motor %d to brake" % motor)
  if init_okay: i2c.write_byte_data(s.MOTOR_ADDRESSES[motor], 0, byte)

#Test code
if __name__ == "__main__":
    s.init()
    s.setup_logger("motors")
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

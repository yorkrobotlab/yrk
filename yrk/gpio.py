#!/usr/bin/python
#
# York Robotics Kit Python API
# Version 0.2
# Functions for the GPIO expansion and switches outputs from U13 [PCA9555 16-way GPIO]
# Datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9555.pdf
# James Hilder, York Robotics Laboratory, Jan 2020

"""
.. module:: gpio
   :synopsis: Functions for the I2C GPIO Expansion

.. moduleauthor:: James Hilder <github.com/jah128>

The YRL040 PCB contains two PCA9555 16-bit GPIO expanders, one is connected to
the switches [see ``switch`` module], the other provides 8 user GPIO pins on
a 0.1" pitch 4x2 header.  It also uses 4 GPIO input to detect fault conditions
in the four H-bridge motor drivers, and 1 more to provide the motor fault LED
output.  Another input is used to provide the "kill switch" input, which might
be configured to stop all running code etc.

The final pair of GPIO outputs are used to provide an NMOS-driven 5V and 12V
switched output.  These outputs can be used to turn on a DC load [fused at 1A],
but note the negative terminal is not a common ground with the main PCB and
should not be connected.  This makes the output most suited to drive things such
as a buzzer or lamp, or potentially a relay.  The 5V output is connected to the
5V_AUX supply [though will be slightly under 5V due to the FET losses]; the 12V
is connected to V_IN.

PCA5555 GPIO Expander Datasheet:
https://www.nxp.com/docs/en/data-sheet/PCA9555.pdf

"""
import yrk.settings as s, yrk.utils as utils
import smbus2 #I2C function
import time, logging, os

# On YRL040, GPIO 6 is interrupt in
# IO0-0 : IO0-7  USER 8-BIT GPIO CONNECTOR [I/O]
# IO1-0 : IO1-3  MOTOR FAULT [I]
# IO1-4          5V SWITCHED OUTPUT [O]
# IO1-5          12V SWITCHED OUTPUT [O]
# IO1-6          MOTOR FAULT LED [O]
# IO1-7          KILL SWITCH [I]

try:
  i2c = smbus2.SMBus(s.YRL040_BUS)
  init_okay = True
except FileNotFoundError:
  logging.error("[gpio.py]     : Cannot access /dev/i2c-%d"  % (s.YRL040_BUS))
  init_okay = False
  s.BUS_ERROR = True


switched_output_5V = False
switched_output_12V = False
motor_fault_led = True #Inverted output [True=LED off]

def setup_user_gpio():
    """An initialisation function for the PCA9555 GPIO Expander for the user GPIO and motor fault detection

    Sets pins IO0_0 : IO0_7 based on values from settings.py
    Sets pins IO1_0 : IO1_3 as inverted inputs for motor fault detection
    Sets pins IO1_4 : IO1_6 as outputs [off] for switched outputs and motor fault LED
    Sets pin IO1_7 as inverted input for kill switch

    """

    #Set input 0.0-0.7 and 1.0-1.2 as inverted inputs, 1.3-1.7 as outputs [for LEDs]
    if init_okay:
        utils.i2c_lock()
        i2c.write_byte_data(s.USER_GPIO_ADDRESS, 0x02, s.USER_GPIO_OUTPUT_STATE)    #Output register [if]
        i2c.write_byte_data(s.USER_GPIO_ADDRESS, 0x03, 0x40)
        i2c.write_byte_data(s.USER_GPIO_ADDRESS, 0x04, s.USER_GPIO_INVERTED)        #Polarity inversion
        i2c.write_byte_data(s.USER_GPIO_ADDRESS, 0x05, 0x8F)
        i2c.write_byte_data(s.USER_GPIO_ADDRESS, 0x06, s.USER_GPIO_MODE)            #Mode register []
        i2c.write_byte_data(s.USER_GPIO_ADDRESS, 0x07, 0x8F)
        utils.i2c_unlock()

def update_switched_gpio_outputs():
    """Sends i2c command to set the switched outputs based on stored variables"""

    if init_okay:
        utils.i2c_lock()
        i2c.write_byte_data(s.USER_GPIO_ADDRESS, 0x03, (switched_output_5V << 4) + (switched_output_12V << 5) + (motor_fault_led << 6))
        utils.i2c_unlock()

def set_switched_output_5V(state):
    """A function to enable the 5V switched output

    Note the PD will actually be a little under 5V.  V- is not GND and should
    not be connected to ground.

    Args:
        state (bool): Enable or disable the 5V output

    """

    global switched_output_5V
    switched_output_5V = state
    update_switched_gpio_outputs()

def set_switched_output_12V(state):
    """A function to enable the 12V switched output

    Note the PD will actually be a little under Vin.  V- is not GND and should
    not be connected to ground.

    Args:
        state (bool): Enable or disable the 12V output

    """

    global switched_output_12V
    switched_output_12V = state
    update_switched_gpio_outputs()

def set_motor_fault_led(state):
    """A function to enable or disable the motor fault LED

    Args:
        state (bool): Enable or disable the motor fault LED

    """

    global motor_fault_led
    motor_fault_led = not state
    update_switched_gpio_outputs()

def read_user_gpio():
    """A function to read the input registers of the user GPIO Expander

    Returns:
        int: 16-bit value indicating user GPIO states.

    """

    if not init_okay:
        logging.error("Call to read user GPIO failed")
        return 0
    utils.i2c_lock()
    ret_val = i2c.read_word_data(s.USER_GPIO_ADDRESS, 0x00)
    utils.i2c_unlock()
    return ret_val

def read_motor_fault():
    return ((read_user_gpio() & 0x0F00) >> 8)


def init():
    setup_user_gpio()

#Test code
if __name__ == "__main__":
    s.init()
    s.setup_logger("gpio")
    logging.info("York Robotics Kit: User GPIO test code")
    try:
      setup_user_gpio()
      while(True):
        logging.info("Motor fault LED on, 12V supply off")
        set_motor_fault_led(True);
        set_switched_output_12V(False);
        time.sleep(0.5);
        logging.info("5V supply on, motor fault LED off")
        set_switched_output_5V(True);
        set_motor_fault_led(False);
        time.sleep(0.5);
        logging.info("12V supply on, 5V supply off")
        set_switched_output_12V(True);
        set_switched_output_5V(False);
        time.sleep(0.5);
    except IOError:
      logging.debug("No user GPIO expander detected at address 0x%X on bus i2c_%d" % (s.USER_GPIO_ADDRESS,s.YRL040_I2CBUS))

# Notes on PCA9555 [from datasheet]
# The command byte is the first byte to follow the address byte during a write transmission.
# It is used as a pointer to determine which of the following registers will be written or read.
#
# Command Register
# 0 Input port 0
# 1 Input port 1
# 2 Output port 0
# 3 Output port 1
# 4 Polarity Inversion port 0 [1=Inverted]
# 5 Polarity Inversion port 1
# 6 Configuration port 0 [1=Input [pull-up], 0=Output]
# 7 Configuration port 1

#!/usr/bin/python
#
# York Robotics Kit Python API
#
# Version 0.1
#
# Functions for the GPIO expansion and switches outputs from U13 [PCA9555 16-way GPIO]
#
# Datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9555.pdf
# James Hilder, York Robotics Laboratory, Oct 2019

import yrk.settings as s
import smbus2 #I2C function
import time, logging, os

# On YRL040, GPIO 6 is interrupt in
# IO0-0 : IO0-7  USER 8-BIT GPIO CONNECTOR [I/O]
# IO1-0 : IO1-3  MOTOR FAULT [I]
# IO1-4          5V SWITCHED OUTPUT [O]
# IO1-5          12V SWITCHED OUTPUT [O]
# IO1-6          MOTOR FAULT LED [O]
# IO1-7          KILL SWITCH [I]

i2c = smbus2.SMBus(s.YRL040_BUS)
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
    i2c.write_byte_data(s.USER_GPIO_ADDRESS, 0x02, s.USER_GPIO_OUTPUT_STATE)    #Output register [if]
    i2c.write_byte_data(s.USER_GPIO_ADDRESS, 0x03, 0x00)
    i2c.write_byte_data(s.USER_GPIO_ADDRESS, 0x04, s.USER_GPIO_INVERTED)        #Polarity inversion
    i2c.write_byte_data(s.USER_GPIO_ADDRESS, 0x05, 0x8F)
    i2c.write_byte_data(s.USER_GPIO_ADDRESS, 0x06, s.USER_GPIO_MODE)            #Mode register []
    i2c.write_byte_data(s.USER_GPIO_ADDRESS, 0x07, 0x8F)


def update_switched_gpio_outputs():
    """Sends i2c command to set the switched outputs based on stored variables"""

    i2c.write_byte_data(s.USER_GPIO_ADDRESS, 0x03, (switched_output_5V << 4) + (switched_output_12V << 5) + (motor_fault_led << 6))

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

    return (i2c.read_word_data(s.USER_GPIO_ADDRESS, 0x00))

def read_motor_fault():
    return ((read_user_gpio() & 0x0F00) >> 8)


def init():
    setup_user_gpio()

#Test code
if __name__ == "__main__":
    s.init()
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

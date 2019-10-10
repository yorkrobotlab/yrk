#!/usr/bin/python
#
# York Robotics Kit Python API
# Version 0.1
# Functions for the switches and buttons connected to U4 [PCA9555 16-way GPIO]
# Datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9555.pdf
# James Hilder, York Robotics Laboratory, Oct 2019

"""
.. module:: switch
   :synopsis: Functions for the switches and buttons

.. moduleauthor:: James Hilder <github.com/jah128>

This module contains function for the switch GPIO expander [one of two PCA9555
GPIO expanders on the YRL040 PCB].  The switch module, at the bottom of the
YRL040 PCB, contains a 4-way DIP switch with associated LEDs, a 5-way
directional switch and 2 push-buttons at either side.  The green power LED is
also connected to the GPIO expander.

PCA5555 GPIO Expander Datasheet:
https://www.nxp.com/docs/en/data-sheet/PCA9555.pdf

"""
import yrk.settings as s
import smbus2 #I2C function
import time, logging, os

# On YRL040, GPIO 5 is interrupt in
# IO0-0 : IO0-3  4-WAY DIP SWITCH [I]
# IO0-4 : IO0-7  NAV SWITCH U-D-L-R [I]
# IO1-0          NAV SWITCH CENTER [I]
# IO1-1          PUSH BUTTON 1 [I]
# IO1-2          PUSH BUTTON 2 [I]
# IO1-3 : IO1-6  4-WAY DIP LEDS [O]
# IO1-7          POWER FAULT LED [O]

switch_gpio_output_byte = 0
i2c = smbus2.SMBus(s.YRL040_BUS)

def update_switch_gpio_output(new_states):
    """A function to update the output pins on the PCA9555 GPIO Expander

    Args:
        new_states (int): A 1-byte value [MSB=1_7, Green LED].

    """

    OUTPUT_PORT = 0x03
    global switch_gpio_output_byte
    switch_gpio_output_byte = new_states
    i2c.write_byte_data(s.SWITCH_GPIO_ADDRESS, OUTPUT_PORT, switch_gpio_output_byte)


def setup_switch_gpio():
    """An initialisation function for the PCA9555 GPIO Expander controlling the switches

    Sets pins IO0_0 : IO0_7 and IO1_0 : IO1_2 as inverted inputs [ie 0V = True]
    Sets pins IO1_3 : IO1_7 as outputs.  IO1_3 : IO1_6 are DIP LEDs, IO1_7 is green power LED

    """

    #Set input 0.0-0.7 and 1.0-1.2 as inverted inputs, 1.3-1.7 as outputs [for LEDs]
    i2c.write_byte_data(s.SWITCH_GPIO_ADDRESS, 0x04, 0xFF)                        #Polarity inversion
    i2c.write_byte_data(s.SWITCH_GPIO_ADDRESS, 0x05, 0x07)
    i2c.write_byte_data(s.SWITCH_GPIO_ADDRESS, 0x06, 0xFF)                        #IO State
    i2c.write_byte_data(s.SWITCH_GPIO_ADDRESS, 0x07, 0x07)


#Return [11 bit int] value for input registers
def read_input_registers():
    """A function to read the state of the switches at the bottom of the YRL040 PCB

    Returns:
        int: 11-bit value indicating switch states.

    """

    return (i2c.read_word_data(s.SWITCH_GPIO_ADDRESS, 0x00) & 0x7FF)

#Return [nibble] value for 4-way dip-switch
def read_dip_switch():
    """A function to read the state of the 4-way DIP switch

    Returns:
        int: 4-bit value indicating switch states

    """

    return read_input_registers() % 16

def set_dip_leds(nibble):
    """A function to set the state of the yellow LEDs above the DIP switch

    Args:
        nibble (int): A four-bit value indicating the target state of the LEDs

    """

    update_switch_gpio_output((switch_gpio_output_byte & 0x87) + (nibble << 3))

def set_power_green_led(state):
    """A function to set the state of the green power LED at top of YRL040 PCB

    Args:
        state (bool): Enable or disable the LED

    """

    update_switch_gpio_output((switch_gpio_output_byte & 0x78) + (state << 7))

def init():
    setup_switch_gpio()

#Test code
if __name__ == "__main__":
    s.init()
    logging.info("Switch test code")
    try:
      # setup_switch_gpio()
      # logging.info("Switch output:{0:016b}".format(read_input_registers()))
      # set_dip_leds(read_dip_switch())
      while(True):
        setup_switch_gpio()
        logging.info("Switch output:{0:011b}".format(read_input_registers()))
        set_dip_leds(read_dip_switch())
        time.sleep(0.2);
        set_power_green_led(True);
        time.sleep(0.2);
        set_power_green_led(False);
    except IOError:
      logging.debug("No switch GPIO expander detected at address 0x%X on bus i2c_%d" % (s.SWITCH_GPIO_ADDRESS,s.YRL040_I2CBUS))

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

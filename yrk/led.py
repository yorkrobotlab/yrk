#!/usr/bin/python
#
# York Robotics Kit Python API
#
# Version 0.1
#
# Functions for the TCA6507 LED driver [front RGB LEDs]
#
# Datasheet: http://www.ti.com/lit/ds/symlink/tca6507.pdf
# James Hilder, York Robotics Laboratory, Oct 2019

import logging, threading, time, os, smbus2                                                      #General Python imports
import yrk.settings as s
from threading import Timer

body_bus = smbus2.SMBus(s.YRL040_BUS)                                           #The TCA6507 is attached to the YRL040 3.3V I2C Bus
LED_REGISTER = 0x10                                                             #Auto-increment from register 0 on TCA6507

brightness = 4

body_animations =  [ ["Off", [0x00,0x00,0x00,0x44,0x44,0x44,0x44,0x44]],
                ["Red Pulse",[0x01,0x09,0x09,0x44,0x44,0x44,0x44,0x44]],
		["Green Pulse",[0x02,0x12,0x12,0x44,0x44,0x44,0x44,0x44]],
		["Blue Pulse",[0x04,0x24,0x24,0x44,0x44,0x44,0x44,0x44]],
		["Yellow Blink",[0x00,0x12,0x1B,0x22,0x00,0x22,0x00,0x00]],
		["Blue:Green",[0x12,0x36,0x36,0x88,0x00,0x88,0x00,0x00]],
		["Strobe",[0x00,0x7F,0x7F,0x00,0x11,0x11,0x00,0x00]],
                ["Purple Pulse",[0x24,0x09,0x2D,0x12,0x11,0x21,0x00,0x11]],
                ["Red Blink",[0x01,0x09,0x09,0x11,0x11,0x11,0x11,0x11]] ]

solid_colours = [['Off',0x00,0x00],
		 ['Red',0x01,0x08],
		 ['Yellow',0x03,0x18],
		 ['Green',0x02,0x10],
		 ['Cyan',0x06,0x30],
		 ['Blue',0x04,0x20],
		 ['Purple',0x05,0x28],
		 ['White',0x07,0x38],
 		 ['Others',0x40,0x40]  ]

def timed_animation(index, time):
    timer = Timer(time, stop_animation)
    animation(index)
    timer.start()

def get_brightness_int():
    b_int = (brightness << 4) + brightness
    logging.debug("Setting brightness to %X" % b_int)
    return b_int;

def animation(index):
    logging.debug("Playing animation index %d" % index)
    body_bus.write_i2c_block_data(s.RGB_LED_ADDRESS, LED_REGISTER, body_animations[index][1].extend([get_brightness_int(),0x0F,0x11]))

def stop_animation():
    animation(0)

def set_right_colour_solid(index):
  if index < 0 or index > len(solid_colours):
    logging.warning("Requested colour outside range; ignoring.")
  else:
    logging.debug("Setting right LED to %s" % solid_colours[index][0])
    body_bus.write_i2c_block_data(s.RGB_LED_ADDRESS, LED_REGISTER, [solid_colours[index][2],0x00,solid_colours[index][2],0x44,0x44,0x44,0x44,0x00,get_brightness_int(),brightness,0x11])

def set_left_colour_solid(index):
  if index < 0 or index > len(solid_colours):
    logging.warning("Requested colour outside range; ignoring.")
  else:
    logging.debug("Setting left LED to %s" % solid_colours[index][0])
    body_bus.write_i2c_block_data(s.RGB_LED_ADDRESS, LED_REGISTER, [solid_colours[index][1],0x00,solid_colours[index][1],0x44,0x44,0x44,0x44,0x00,get_brightness_int(),brightness,0x11])


def set_right_colour_pulse(index,speed=0x04):                                     #Speed range 0 to 8
  if index < 0 or index > len(solid_colours):
    logging.warning("Requested colour outside range; ignoring.")
  else:
    logging.debug("Setting right LED to pulse %s" % solid_colours[index][0])
    body_bus.write_i2c_block_data(s.RGB_LED_ADDRESS, LED_REGISTER, [0x00,solid_colours[index][2],solid_colours[index][2],speed,speed<<1,speed,speed,speed,get_brightness_int(),0x0F,0x11])

def set_left_colour_pulse(index,speed=0x04):                                  #Speed range 0 to 15
  if index < 0 or index > len(solid_colours):
    logging.warning("Requested colour outside range; ignoring.")
  else:
    logging.debug("Setting left LED to pulse %s" % solid_colours[index][0])
    body_bus.write_i2c_block_data(s.RGB_LED_ADDRESS, LED_REGISTER, [0x00,solid_colours[index][1],solid_colours[index][1],speed,speed<<1,speed,speed,speed,get_brightness_int(),0x0F,0x11])

def set_colour_pulse(index,speed=0x04):                                         #Speed range 0 to 15
  if index < 0 or index > len(solid_colours):
    logging.warning("Requested colour outside range; ignoring.")
  else:
    logging.debug("Setting LEDs to pulse %s" % solid_colours[index][0])
    body_bus.write_i2c_block_data(s.RGB_LED_ADDRESS, LED_REGISTER, [0x00,solid_colours[index][1]+solid_colours[index][2],solid_colours[index][1]+solid_colours[index][2],speed,speed<<1,speed,speed,speed,get_brightness_int(),0x0F,0x11])

def set_colour_solid(index):
  if index < 0 or index > len(solid_colours):
    logging.warning("Requested colour outside range; ignoring.")
  else:
    logging.debug("Setting LEDs to %s" % solid_colours[index][0])
    body_bus.write_i2c_block_data(s.RGB_LED_ADDRESS, LED_REGISTER, [solid_colours[index][1]+solid_colours[index][2],0x00,solid_colours[index][1]+solid_colours[index][2],0x44,0x44,0x44,0x44,0x44,get_brightness_int(),brightness,0xFF])


#Command line test [will run when led.py is run directly]
if __name__ == "__main__":
 logger = logging.getLogger()
 logger.setLevel(logging.DEBUG)
 for i in range(16):
     brightness = i
     #set_left_colour_pulse(5)
     #set_right_colour_solid((i+1) % 8)
     set_left_colour_solid(5)

     time.sleep(0.2);
 os._exit(1)


# LED Driver TI TCA6507
# http://www.ti.com/lit/ds/symlink/tca6507.pdf
# Port 0: LED_1_RED
# Port 1: LED_1_GREEN
# Port 2: LED_1_BLUE
# Port 3: LED_2_RED
# Port 4: LED_2_GREEN
# Port 5: LED_2_BLUE
# Port 6: User LED [top-left expansion pin]

# Registers:
# 0x00: Select0 [BANK1/PWM1 Characteristics]
# 0x01: Select1 [BANK0/PWM0 Characteristics]
# 0x02: Select2 [LED ON with Select0\1 Characteristics]
# 0x03: Fade-ON Register [Split C7:C4 = Bank 1 C3:C0 = Bank 0]
# 0x04: Fully-ON Register [Split C7:C4 = Bank 1 C3:C0 = Bank 0]
# 0x05: Fade-OFF Register [Split C7:C4 = Bank 1 C3:C0 = Bank 0]
# 0x06: Fully-OFF Register 1 [Split C7:C4 = Bank 1 C3:C0 = Bank 0]
# 0x07: Fully-OFF Register 2 [Split C7:C4 = Bank 1 C3:C0 = Bank 0]
# 0x08: Brightness [Split C7:C4 = Bank 1 C3:C0 = Bank 0]
# 0x09: One-Shot \ Master Intensity

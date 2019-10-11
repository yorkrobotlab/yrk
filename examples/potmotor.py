"""
.. module:: potmotor
   :synopsis: Vary motor 0 speed based on pot position

.. moduleauthor:: James Hilder <github.com/jah128>

An simple script showing how to set the speed of motor 0, and the brightness
of the LEDs, proportional to the potentiometer setting.  Attach a motor to
motor port 0 and set pot roughly to middle position before running!

To run::

   python potmotor.py


Use ``Ctrl-C`` to exit, then run::

   python stop.py

to reset motors and leds.

"""

import yrk.adc as adc, yrk.motors as motors, yrk.led as led
import time

#Program code
if __name__ == "__main__":
    while(True):
        #Read the raw value of the pot. 255 is fully left, 0 is fully right
        pot_value = adc.read_adc(6)
        #Set motor 0 speed to be fully backwards [-1] at fully left and forwards [1] at fully right
        motors.set_motor_speed(0,1.0 - (0.007843 * pot_value))
        #Set LED brightness to be proportional to speed.  Brightness range is 0-15.
        led.set_brightness((abs(128-pot_value) + 6) >> 3)
        #Make the LEDs white
        led.set_colour_solid(7)
        #Add a short wait to keep system responsive
        time.sleep(0.01)

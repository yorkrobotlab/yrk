"""
.. module:: potservo
   :synopsis: Vary servo 0 angle based on pot position

.. moduleauthor:: James Hilder <github.com/jah128>

An simple script showing how to set PWM duty cycle (on-time) of one of the
servo outputs, proportional to the potentiometer setting.  Attach a small analog
servo to port 0.  The on-time will be vary between *1.0 mS* and *2.0 mS* as the
potentiometer is rotated, which is a typical range for most analog servos.  The
PWM frequency is set to about 50Hz, again typical for most servo motors.

To run::

   python potservo.py


Use ``Ctrl-C`` to exit

"""

import yrk.adc as adc, yrk.pwm as pwm
import time

#Program code
if __name__ == "__main__":
    pwm.set_pwm_frequency(50)
    while(True):
        #Read the raw value of the pot. 255 is fully left, 0 is fully right
        pot_value = adc.read_adc(6)
        target_period_us = 2010 - (pot_value * 4)
        raw_dutycycle = pwm.calculate_nearest_duty_cycle_to_period(0.000001 * target_period_us)
        print("Duty cycle: %d  Target period: %dus" % (raw_dutycycle, target_period_us))
        pwm.set_duty_cycle_raw(0,raw_dutycycle)
        #Add a short wait to keep system responsive
        time.sleep(0.05)

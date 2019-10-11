"""
.. module:: stop
   :synopsis: Stop and reset everything

.. moduleauthor:: James Hilder <github.com/jah128>

An example script showing how to stop and reset everything.  The following
sequence takes place:


* Call ``motors.stop_all_motors()`` to stop motors
* Call ``led.set_colour_solid(0)`` to turn off RGB LEDs
* Call ``display.clear()`` to clear OLED display

To run::

   python stop.py

"""

import yrk.settings as s,yrk.utils as utils, yrk.adc as adc, yrk.motors as motors
import yrk.power as power, yrk.switch as switch, yrk.gpio as gpio, yrk.led as led
import yrk.display as display
import logging

def stop_all():
    """Stops and resets all actuators"""
    motors.stop_all_motors()
    led.set_colour_solid(0)
    display.clear()

#Program code
if __name__ == "__main__":
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    logging.info("York Robotics Kit: Stop Everything example code")
    stop_all()

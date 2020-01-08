#!/usr/bin/env python

# York Robotics Kit ROS (Python) API
# Version 0.2
# Display server: ROS service for display text on the I2C OLED display module
# James Hilder, York Robotics Laboratory, Jan 2020


"""
.. module:: display_server
   :synopsis: A ROS service for displaying text on the I2C OLED display module

.. moduleauthor:: James Hilder <github.com/jah128>

The ``display_server.py`` script contains the code for the *display_service* ROS service in the *yrk-ros* library.
This provides the functionality to display text, warnings and to clear the I2C OLED module, using functions from the :mod:`yrk.display` core module.

The service message is defined in the file *Display.srv* and contains 5 input fields, shown in the table below, and one response field (*bool* **success**).

======   ===============  ======
Type     Name             Notes
======   ===============  ======
string   message_1
string   message_2        Only used when *number_of_lines==2* and *warning==false*
unit8    number_of_lines  Range 0-2, 0=clear display
bool     wrap_text        Shrinks font, wraps text over 2 lines if needed
bool     warning          Displays ~10 chars text [message_1] with warning icon
======   ===============  ======

The Display service message is processed using the following logic:

* If *number_of_lines*>2 exit with failure
* If *number_of_lines* =0 clear display
* If *warning* is true, display *message_1* as a warning (ignore *number_of_lines*, *wrap_text*, *message_2*)
* Otherwise call 1- or 2- line wrapped or normal functions based on settings

To run outside of launch file::

  rosrun yrk_ros adc_publisher.py

Usage Example::

  rosservice call /display_service "{message_1: '', message_2: '', number_of_lines: 0, wrap_text: false, warning: false}"


.. note::  Use tab completion on rosservice call */display_service* to prepare above macro, works elsewhere too!


"""

# Imports
import rospy
from yrk_ros.srv import Display,DisplayResponse
import yrk.settings as s
import yrk.display as display

#Callback function called when request is made to service
def display_callback(request):
    #Check bounds
    lines= request.number_of_lines
    if(lines > 2):
        print("Warning: Request to display number_of_lines out of bounds: %d" % lines)
        return False
    else:
        if(lines == 0):
            print("Clearing display")
            display.clear()
            return True
        if(request.warning):
            display.warning(request.message_1)
            print("Display warning message %s" % (request.message_1))
            return True
        if(lines == 1):
            print("Display message %s" % (request.message_1))
            if request.wrap_text:
                display.one_line_text_wrapped(request.message_1)
            else:
                display.one_line_text(request.message_1)
        else:
            print("Display message %s | %s" % (request.message_1, request.message_2))
            if request.wrap_text:
                display.two_line_text_wrapped(request.message_1,request.message_2)
            else:
                display.two_line_text(request.message_1,request.message_2)
        return True


# Setup function
def display_server_init():
    # Initialise the node
    rospy.init_node('display_service')
    #Declare service
    service = rospy.Service('display_service',Display,display_callback)
    print ("Display service ready.")
    rospy.spin()

# Calls the switched_output_server_init() function when the file is run
if __name__ == '__main__':
    try:
        display_server_init()
    except rospy.ROSInterruptException:
        pass

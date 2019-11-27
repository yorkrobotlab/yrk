#!/usr/bin/env python

# York Robotics Kit ROS (Python) API
# Version 0.1
# LED server: ROS service for setting the RGB LEDs
# James Hilder, York Robotics Laboratory, Nov 2019


"""
.. module:: led_server
   :synopsis: A ROS service for the RGB LEDs on the York Robotics Kit

.. moduleauthor:: James Hilder <github.com/jah128>

Controls the colour, mode and brightness of the RGB LEDs on the YRK.
The message expects 3 arguments:
uint8 index : Colour index
uint8 brightness:  Brightness value [0-15]

Output is a 'name' string describing LED colour or animation mode

To run::

   rosrun yrk_ros led_server.py

"""

# Imports
import rospy
from yrk_ros.srv import LED,LEDResponse
import yrk.settings as s
import yrk.led as led

#Callback function called when request is made to service
def led_callback(request):
    #Check bounds
    brightness = request.brightness
    if(brightness > 15):
        print("Warning: LED brightness beyond maximum [15]: %d" % brightness)
        brightness = 15
    led.set_brightness(brightness)
    index=request.index
    if(request.animation):
        if(index > len(led.body_animations)):
            print("Error: LED animation index outside valid range [%d]: %d" % (len(led.body_animations),index))
            return "Error"
        name = led.body_animations[index][0]
        print("Setting LEDs to animation %s" % name)
        led.animation(index)
        return name
    else:
        if(index > len(led.solid_colours)):
            print("Error: LED colour index outside valid range [%d]: %d" % (len(led.solid_colours),index))
            return "Error"
        name = led.solid_colours[index][0]
        print("Setting LEDs to colour %s" % name)
        led.set_colour_solid(index)
        return name

# Setup function
def led_server_init():
    # Initialise the node
    rospy.init_node('led_service')
    #Declare service
    service = rospy.Service('led_service',LED,led_callback)
    print ("LED service ready.")
    rospy.spin()

# Calls the switched_output_server_init() function when the file is run
if __name__ == '__main__':
    try:
        led_server_init()
    except rospy.ROSInterruptException:
        pass

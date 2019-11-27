#!/usr/bin/env python

# York Robotics Kit ROS (Python) API
# Version 0.1
# Switched output server: ROS service for enabling the 5V and 12V switched outputs
# James Hilder, York Robotics Laboratory, Nov 2019


"""
.. module:: switched_output_server
   :synopsis: A ROS service for the switched outputs on the York Robotics Kit

.. moduleauthor:: James Hilder <github.com/jah128>

Enables\disables the 5V and 12V switched outputs.

To run::

   rosrun yrk_ros switched_output_server.py

"""

# Imports
import rospy
from yrk_ros.srv import SwitchedOutput,SwitchedOutputResponse
import yrk.settings as s
import yrk.gpio as gpio

#Callback function called when request is made to service
def switched_output_callback(request):
    if(request.output == 0):
        print("Setting 12V switched output to %s" % request.enable)
        gpio.set_switched_output_12V(request.enable)
    elif(request.output == 1):
        print("Setting 5V switched output to %s" % request.enable)
        gpio.set_switched_output_5V(request.enable)
    else:
        print("Invalid request to switched output service %d" % request.output)
        return False
    return True

# Setup function
def switched_output_server_init():
    # Initialise the node
    rospy.init_node('switched_output_service')
    #Declare service
    service = rospy.Service('switched_output_service',SwitchedOutput,switched_output_callback)
    print ("Switched output service ready.")
    gpio.setup_user_gpio()
    gpio.set_switched_output_5V(False)
    gpio.set_switched_output_12V(False)
    rospy.spin()

# Calls the switched_output_server_init() function when the file is run
if __name__ == '__main__':
    try:
        switched_output_server_init()
    except rospy.ROSInterruptException:
        pass

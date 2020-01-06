#!/usr/bin/env python

# York Robotics Kit ROS (Python) API
# Version 0.2
# Motor server: ROS service for setting the 4x TI I2C Motor Drivers on the York Robotics Kit
# James Hilder, York Robotics Laboratory, Jan 2020


"""
.. module:: motor_server
   :synopsis: A ROS service for the TI DRV8830 motor drivers on the York Robotics Kit

.. moduleauthor:: James Hilder <github.com/jah128>

Sets the speed and brake mode [high\low impedance] for the motors on the York
Robotics Kit.
The message expects 3 arguments:
uint8 motor_index : Motor index [0-3]
float32 speed:  Motor speed [-1.0 to 1.0, 0=brake or coast]
bool brake_mode:  If speed=0.0 and brake_mode=true, will force low-impedance brake

Returns:
bool success

Output is a 'name' string describing LED colour or animation mode

To run::

   rosrun yrk_ros led_server.py

"""

# Imports
import rospy
from yrk_ros.srv import Motor,MotorResponse
import yrk.settings as s
import yrk.motors as motors

#Callback function called when request is made to service
def motor_callback(request):
    #Check bounds
    motor_id = request.motor_index
    if(motor_id > 3):
        print("Warning: Request to motor index out of bounds: %d" % motor_id)
        return False
    else:
        speed = request.speed
        if(speed < -1.0):
            speed = -1.0
            print("Warning: Request to speed outside bounds, setting to -1.0: %f" % speed)
        if(speed > 1.0):
            speed = 1.0
            print("Warning: Request to speed outside bounds, setting to 1.0: %f" % speed)
        if speed == 0 and request.brake_mode:
            print("Setting motor %d to brake" % (motor_id))
            motors.brake_motor(motor_id)
        else:
            print("Setting motor %d to speed %f" % (motor_id,speed))
            motors.set_motor_speed(motor_id,speed)
        return True


# Setup function
def motor_server_init():
    # Initialise the node
    rospy.init_node('motor_service')
    #Declare service
    service = rospy.Service('motor_service',Motor,motor_callback)
    print ("Motor service ready.")
    rospy.spin()

# Calls the switched_output_server_init() function when the file is run
if __name__ == '__main__':
    try:
        motor_server_init()
    except rospy.ROSInterruptException:
        pass

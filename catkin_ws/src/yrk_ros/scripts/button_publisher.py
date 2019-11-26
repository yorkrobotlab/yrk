#!/usr/bin/env python

# York Robotics Kit ROS (Python) API
# Version 0.1
# Button Publisher: ROS publisher service for the switches and buttons
# James Hilder, York Robotics Laboratory, Nov 2019


"""
.. module:: button_publisher
   :synopsis: A ROS publisher service for the buttons on the York Robotics Kit

.. moduleauthor:: James Hilder <github.com/jah128>

Reads the values from the buttons and switches and publishes as a rostopic.

To run::

   rosrun yrk_ros button_publisher.py

"""

# Imports
import rospy
import yrk_ros.msg
import yrk.settings as s
import yrk.switch as switch

# Sender function
def button_publisher():

	# Declare a topic called /data with String type
	pub = rospy.Publisher('button_publisher', yrk_ros.msg.switch_message, queue_size=3)

	# Initialise the node
	rospy.init_node('button_publisher')

	# Loop five times per second and publish a message to the topic
	rate = rospy.Rate(s.ROS_SWITCH_PUBLISHER_RATE)
	while not rospy.is_shutdown():
		msg = yrk_ros.msg.switch_message()
		switch_register = switch.read_input_registers()
		msg.push0 = switch_register & 0x200
		msg.push1 = switch_register & 0x400
		msg.center = switch_register & 0x100
		msg.dip0 = switch_register & 0x001
		msg.dip1 = switch_register & 0x002
		msg.dip2 = switch_register & 0x004
		msg.dip3 = switch_register & 0x008
		msg.up = switch_register & 0x010
		msg.down = switch_register & 0x020
		msg.left = switch_register & 0x040
		msg.right = switch_register & 0x080
		rospy.loginfo("Sending switch message")
		pub.publish(msg)
		rate.sleep()

# Calls the button_publisher() function when the file is run
if __name__ == '__main__':
	try:
		button_publisher()
	except rospy.ROSInterruptException:
		pass

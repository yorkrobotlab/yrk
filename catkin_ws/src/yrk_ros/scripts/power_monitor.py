#!/usr/bin/env python

# York Robotics Kit ROS (Python) API
# Version 0.1
# Power Monitor: ROS publisher service for YRL039 power supply board
# James Hilder, York Robotics Laboratory, Nov 2019


"""
.. module:: power_monitor
   :synopsis: A ROS publisher service for power statistics from the YRL039 board

.. moduleauthor:: James Hilder <github.com/jah128>

Reads the values from the Atmega328 microcontroller on the YRL039 power supply
board and publishes as a ROS topic using the yrk_ros.msg.power_status message
format.

To run::

   rosrun yrk_ros power_monitor.py

"""

# Imports
import rospy
import yrk_ros.msg
import yrk.settings as s
import yrk.power as power
import yrk.utils as utils

# Sender function
def power_status_publisher():

	# Declare a topic called /data with String type
	pub = rospy.Publisher('power_status', yrk_ros.msg.power_status, queue_size=3)

	# Initialise the node
	rospy.init_node('power_status_publisher')

	# Loop once per second and publish a message to the topic
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		power.read_all_values()
		msg = yrk_ros.msg.power_status()
		msg.v_battery = power.get_battery_voltage()
		msg.v_pi = power.get_pi_voltage()
		msg.v_aux = power.get_aux_voltage()
		msg.i_pi = power.get_pi_current()
		msg.i_aux = power.get_aux_current()
		msg.pcb_temp = power.get_pcb_temperature()
		msg.cpu_temp = utils.get_cpu_temp()
		#rospy.loginfo("Sending power: " + msg)
		pub.publish(msg)
		rate.sleep()

# Calls the power_status_publisher() function when the file is run
if __name__ == '__main__':
	try:
		power_status_publisher()
	except rospy.ROSInterruptException:
		pass

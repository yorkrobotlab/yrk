#!/usr/bin/env python

# York Robotics Kit ROS (Python) API
# Version 0.2
# ADC Publisher: ROS publisher service for the ADC
# James Hilder, York Robotics Laboratory, Jan 2020


"""
.. module:: adc_publisher
   :synopsis: A ROS publisher service for the ADC on the York Robotics Kit

.. moduleauthor:: James Hilder <github.com/jah128>

Reads the values from the 8-port ADC and publish as a rostopic.

To run::

   rosrun yrk_ros adc_publisher.py

"""

# Imports
import rospy
import yrk_ros.msg
import yrk.settings as s
import yrk.adc as adc

# Sender function
def adc_publisher():

	# Declare a topic called /data with String type
	pub = rospy.Publisher('adc_publisher', yrk_ros.msg.adc_message, queue_size=3)

	# Initialise the node
	rospy.init_node('adc_publisher')

	# Loop five times per second and publish a message to the topic
	rate = rospy.Rate(s.ROS_ADC_PUBLISHER_RATE)
	while not rospy.is_shutdown():
		msg = yrk_ros.msg.adc_message()
		msg.a0 = adc.read_adc(0)
		msg.a1 = adc.read_adc(1)
		msg.a2 = adc.read_adc(2)
		msg.a3 = adc.read_adc(3)
		msg.a4 = adc.read_adc(4)
		msg.a5 = adc.read_adc(5)
		msg.a6 = adc.read_adc(6)
		msg.a7 = adc.read_adc(7)
		#rospy.loginfo("Sending adc message")
		pub.publish(msg)
		rate.sleep()

# Calls the power_status_publisher() function when the file is run
if __name__ == '__main__':
	try:
		adc_publisher()
	except rospy.ROSInterruptException:
		pass

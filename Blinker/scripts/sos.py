#!/usr/bin/env python3

from __future__ import print_function

#Python Headers
import math
import os
from time import sleep

# ROS Headers
import rospy

# GEM PACMod Headers
from std_msgs.msg import Header
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, VehicleSpeedRpt

class Node():
	interval = 115000
	sleep_timer = 0.3

	def __init__(self):

		self.rate = rospy.Rate(10)

		self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size = 1)
		# self.turn_cmd = PositionWithSpeed()
		self.turn_cmd = PacmodCmd()
		self.turn_cmd.ui16_cmd = 1

	def blinkLight(self, startup_multiplier=1, direction=1):
		counter = 0

		while not rospy.is_shutdown() and counter < self.interval*startup_multiplier:
			# self.turn_pub.publish(self.turn_cmd)
			self.turn_cmd.ui16_cmd = direction
			self.turn_pub.publish(self.turn_cmd)
			self.rate.sleep
			counter +=1 

		self.turn_cmd.ui16_cmd = 1
		self.turn_pub.publish(self.turn_cmd)
		self.rate.sleep

		# ends with sleep
		sleep(self.sleep_timer)

	def run(self):
		# 2 turns left
		# 1 turns it off
		# 0 turns right
		sos_list = [2,2,2,0,0,0,2,2,2]

		is_startup = True
		for i in range(4):
			print("iteration #{}".format(i))
			for sign in sos_list:
				if is_startup:
					self.blinkLight(startup_multiplier=3, direction=sign)
					is_startup = False
				else:
					self.blinkLight(direction=sign)
			
			sleep(2)

if __name__ == '__main__':
	rospy.init_node('sos_node', anonymous=True)
	node = Node()
	node.run()
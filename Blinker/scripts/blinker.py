#!/usr/bin/env python3

#================================================================
# File name: gem_gnss_pp_tracker_pid.py                                                                  
# Description: gnss waypoints tracker using pid and pure pursuit                                                                
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 08/02/2021                                                                
# Date last modified: 08/15/2022                                                          
# Version: 1.0                                                                   
# Usage: rosrun gem_gnss gem_gnss_pp_tracker.py                                                                      
# Python version: 3.8                                                             
#================================================================

from __future__ import print_function

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal

# ROS Headers
import alvinxy.alvinxy as axy # Import AlvinXY transformation module
import rospy

# GEM Sensor Headers
from std_msgs.msg import String, Bool, Float32, Float64
from novatel_gps_msgs.msg import NovatelPosition, NovatelXYZ, Inspva

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt


class Blink(object):
    def __init__(self):
        # GEM vechile turn signal control
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=10)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1 # None
        self.turn_pub.publish(self.turn_cmd)

    def flashLeft(self, n):
        for x in range(n):
            self.turn_cmd.ui16_cmd = 2
            self.turn_pub.publish(self.turn_cmd)
            self.turn_cmd.ui16_cmd = 1
            self.turn_pub.publish(self.turn_cmd)
            print("flashed left")

    def flashRight(self, n):
        for x in range(n):
            self.turn_cmd.ui16_cmd = 0
            self.turn_pub.publish(self.turn_cmd)
            self.turn_cmd.ui16_cmd = 1
            self.turn_pub.publish(self.turn_cmd)
            print("flashed right")

    def blinkSOS(self):
        rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        self.flashLeft(3)
        self.flashRight(3)
        self.flashLeft(3)
        rate.sleep()

def blink():

    rospy.init_node('gnss_pp_node', anonymous=True)
    pp = Blink()

    try:
        pp.blinkSOS()
        print("done sending msgs")

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    blink()



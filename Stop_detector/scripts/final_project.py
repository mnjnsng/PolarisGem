#!/usr/bin/env python3

import os, sys

import roslib
import rospy
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
from matplotlib import pyplot as plt
import numpy as np

brake_time = 0.5

class Detector:
    def __init__(self):
        # initialize stop sign detector
        self.stop_data = cv2.CascadeClassifier("/home/gem/demo_ws/src/final_project/scripts/stop_data.xml")
        print("loaded stop_data.xml")

    def stop_sign_exists(self, img):
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
         
        found = self.stop_data.detectMultiScale(img_gray,  
                                   minSize =(20, 20))
        amount_found = len(found)
        return amount_found > 0

class Brake_Node:
    def __init__(self):
        self.bridge = CvBridge()
        self.detector = Detector()
        # self.image_sub = rospy.Subscriber("/mako_1/mako_1/image_raw", Image, self.callback)
        self.image_sub = rospy.Subscriber("/zed2/zed_node/rgb_raw/image_raw_color", Image, self.callback)


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            sign_exists = self.detector.stop_sign_exists(cv_image)

            if sign_exists:
                rospy.loginfo("Applying brakes ...")
                pub_enable.publish(True)
                pub_brake.publish(f64_cmd=1, enable=True)
                rospy.loginfo("sent brake command")
                t0 = time.time()
                time.sleep(brake_time)
                pub_brake.publish(f64_cmd=0.0, enable=True)
                rospy.loginfo("cancelled brake command, %.3f sec" % (time.time() - t0))
            else:
                print("No stop sign")
        except CvBridgeError as e:
            print(e)


rospy.init_node("brake_node", anonymous=True)

pub_enable = rospy.Publisher("/pacmod/as_rx/enable",Bool, queue_size=1)
pub_brake = rospy.Publisher("/pacmod/as_rx/brake_cmd",PacmodCmd, queue_size=1)

ic = Brake_Node()
pub_enable.publish(True)

try:
    rospy.spin()
except KeyboardInterrupt:
    cv2.destroyAllWindows()
    print("Shutting down")

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
# import imutils

brake_time = 0.2

class Detector:
    def __init__(self):
        # initialize the HOG descriptor/person detector
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def pedestrian_exists(self, image):
        # image = imutils.resize(image, width=min(400, image.shape[1]))
        # image = cv2.resize(image,(image.shape[0]/400,400))
        (rects, weights) = self.hog.detectMultiScale(
            image, winStride=(4, 4), padding=(8, 8), scale=1.05
        )
        threshold = 0.75
        for weight in weights:
            if weight > threshold:
                return True
        return False


class Brake_Node:
    def __init__(self):
        self.bridge = CvBridge()
        self.detector = Detector()
        # self.image_sub = rospy.Subscriber("/mako_1/mako_1/image_raw", Image, self.callback)
        self.image_sub = rospy.Subscriber("/zed2/zed_node/rgb_raw/image_raw_color", Image, self.callback)


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            person_exists = self.detector.pedestrian_exists(cv_image)
            if person_exists:
                rospy.loginfo("Applying brakes ...")
                pub_enable.publish(True)
                pub_brake.publish(f64_cmd=1, enable=True)
                rospy.loginfo("sent brake command")
                t0 = time.time()
                time.sleep(brake_time)
                pub_brake.publish(f64_cmd=0.0, enable=True)
                rospy.loginfo("cancelled brake command, %.3f sec" % (time.time() - t0))
            else:
                print("No pedestrian")
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

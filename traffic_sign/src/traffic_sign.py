#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int64MultiArray
from cv_bridge import CvBridge

class TrafficSign() :

    def __init__(self) :
        rospy.init_node("traffic_sign")
        self.bridge = CvBridge()

        self.traffic_pub = rospy.Publisher("/traffic_light", Int64MultiArray, queue_size=1)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.imageCallback)


        self.traffic_signal = 0

        self.initialized = False
        rospy.spin()
        

    def imageCallback(self, _data) :
        cv2_image = self.bridge.compressed_imgmsg_to_cv2(_data)                                                                                  
        cv2_image = cv2_image[100:200, 140:500]
        cv2_image = cv2.resize(cv2_image, (700,700))

        hsv_red = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)
        hsv_green = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)

        lower_r01 = np.array([0, 100, 100])
        upper_r01 = np.array([10, 255, 255])
        masked_r01 = cv2.inRange(hsv_red, lower_r01, upper_r01)

        lower_r02 = np.array([170, 100, 100])
        upper_r02 = np.array([180, 255, 255]) 
        masked_r02 = cv2.inRange(hsv_red, lower_r02, upper_r02)

        lower_g = np.array([50, 70, 70]) 
        upper_g = np.array([80, 255, 255])

        masked_red = masked_r01 + masked_r02
        masked_green = cv2.inRange(hsv_green, lower_g, upper_g)

        red_cnt=np.count_nonzero(masked_red)
        green_cnt=np.count_nonzero(masked_green)
        

        msg = Int64MultiArray()
        msg.data = [red_cnt, green_cnt]

        self.traffic_pub.publish(msg)

if __name__ == "__main__":
    traffic = TrafficSign()
 

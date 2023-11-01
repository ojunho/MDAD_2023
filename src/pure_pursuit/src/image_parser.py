#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:
    def __init__(self):

        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

        self.crop_pts = np.array(
            [[
                [0,0], # 왼아
                [0,0], # 왼위
                [0,0], # 오위
                [0,0]  # 오아
            ]]
        )
    
    # roi 범위 밖의 이미지를 안 보이게 처리
    def mask_roi(self, img):

        h = img.shape[0]
        w = img.shape[1]
        
        if len(img.shape) == 3:

            # image shape : [h, w, 3]

            c = img.shape[2]
            mask = np.zeros((h, w, c), dtype=np.uint8)

            mask_value = (255, 255, 255)

        else:

            # binarized image or grayscale image : [h, w]

            mask = np.zeros((h, w), dtype=np.uint8)

            mask_value = (255)

        cv2.fillPoly(mask, self.crop_pts, mask_value)

        mask = cv2.bitwise_and(mask, img)

        return mask

    def callback(self, msg):
        try:
            np_arr = np.fromstring(msg.data, np.uint8)
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)

        img_hsv = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2HSV)

        # H, S, V
        lower_wlane = np.array([0,0,0])
        upper_wlane = np.array([30, 60, 255])

        # 범위 내의 hsv만 255, 나머진 0으로 binarize
        img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)

        img_wlane = cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)

        img_concat = np.concatenate([self.img_bgr, img_hsv, img_wlane], axis=1)

        cv2.imshow("Image window", self.img_bgr)
        cv2.imshow("Image window", img_concat)
        cv2.waitKey(1)


if __name__ == '__main__':

    rospy.init_node('image_parser', anonymous=True)

    image_parser = IMGParser()

    rospy.spin() 
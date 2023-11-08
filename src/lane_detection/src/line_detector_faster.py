#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from warper import Warper
from slide_window_faster import SlideWindow
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image, Imu
from std_msgs.msg import Float64, Int64, Bool
from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, GPSMessage, CtrlCmd
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path, Odometry
from math import *


class Controller() :

    def __init__(self) :
        rospy.init_node("line_detector")


        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.Subscriber("/imu", Imu, self.ImuCB) ## Vehicle Status Subscriber

        self.bridge = CvBridge()
        self.warper = Warper()
        self.slidewindow = SlideWindow()
        self.ctrl_cmd_msg = CtrlCmd()
        self.odom_msg = Odometry()


        self.original_img = []
        self.yaw = 0.0
        self.waypoint = 0


        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        self.waypoint_sub = rospy.Subscriber("/waypoint", Int64, self.waypointCB)
        self.mission_sub = rospy.Subscriber("/mission", Bool, self.mission_callback)
        self.ctrl_cmd_pub = rospy.Publisher('/s_ctrl_cmd', CtrlCmd, queue_size=1)
    

        # slide window return variable initialization
        self.slide_img = None 
        self.slide_x_location = 0.0
        self.current_lane_window = ""

        self.initialized = False # image_callback

        self.error_lane = 0
        self.steering_val = 0
        self.mission = False

        #trash
        self.epoch = 0

        


        # gps 신호 받기 -> gps 끊길 때만 연산
        self.original_longitude = 0.0
        self.original_latitude = 0.0

        # For test
        self.ctrl_cmd_msg.longlCmdType = 2
        self.motor_msg = 5.0
        self.servo_msg = 0.0 
        self.brake_msg = 0.0
        rate = rospy.Rate(20) # 30hz

        
        while not rospy.is_shutdown():
            
            self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
            rate.sleep()
        
        rospy.spin()
    
    # def runnung(self, _event) :

    def mission_callback(self, msg) :
        self.mission = msg.data
        # #print('lane CB', self.mission)

    def waypointCB(self, msg) :
        self.waypoint = msg.data
        # #print('way', self.waypoint)
        
        
    def gpsCB(self, msg): 
        #UTMK
        self.original_longitude = msg.longitude
        self.original_latitude = msg.latitude
    
    def ImuCB(self, msg) :
        self.odom_msg.pose.pose.orientation.x = msg.orientation.x
        self.odom_msg.pose.pose.orientation.y = msg.orientation.y
        self.odom_msg.pose.pose.orientation.z = msg.orientation.z
        self.odom_msg.pose.pose.orientation.w = msg.orientation.w

        quaternion = (self.odom_msg.pose.pose.orientation.x,self.odom_msg.pose.pose.orientation.y,self.odom_msg.pose.pose.orientation.z,self.odom_msg.pose.pose.orientation.w)
        _, _, self.yaw =  euler_from_quaternion(quaternion)
        self.yaw = degrees(self.yaw)

        self.epoch += 1
        if self.epoch % 100 == 0:
            print(self.yaw)

    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # 마우스 왼쪽 버튼 클릭 이벤트 감지
            print(f'클릭한 좌표: ({x}, {y})')


    def image_callback(self, _data) :
        if (self.original_longitude  <= 1 and self.original_latitude <=  1):
            cv2_image = self.bridge.compressed_imgmsg_to_cv2(_data)

            hsv_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)

            lower_lane = np.array([0, 0, 126]) #0 46 170 / white lane: 0 0 126
            upper_lane = np.array([255, 64, 180]) #79 255 255 / white lane : 255, 64, 180


            lane_mask = cv2.inRange(hsv_image, lower_lane, upper_lane)

            ksize = 5
            blur_lane = cv2.GaussianBlur(lane_mask, (ksize, ksize), 0)

            _, lane_image = cv2.threshold(blur_lane, 200, 255, cv2.THRESH_BINARY)

            warper_image = self.warper.warp(lane_image)
            cv2.imshow("warper", warper_image)
            self.slide_img, self.slide_x_location, self.current_lane_window = self.slidewindow.slidewindow(warper_image, self.yaw)

            cv2.imshow("slide_img", self.slide_img)
            cv2.setMouseCallback('slide_img', self.click_event)

            gray_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY)
            lower_c = np.array([0, 156, 190]) 
            upper_c = np.array([219, 190, 255])

            s_image = cv2.inRange(cv2_image, lower_c, upper_c)

            s_mask = cv2.inRange(s_image, 150, 255)
            blur_curve = cv2.GaussianBlur(s_mask, (ksize, ksize), 0)
            _, s_img = cv2.threshold(blur_curve, 200, 255, cv2.THRESH_BINARY)
            warper_s = self.warper.warp(s_img)

            self.curve_img, self.angle = self.slidewindow.curve(warper_s)

            cv2.imshow("out", self.curve_img)

            if self.yaw <= -30 :
                self.slide_x_location += 80
            cv2.waitKey(1)

            self.error_lane = 320 - self.slide_x_location 

            if self.slide_x_location < 270 or self.slide_x_location > 370:
                self.motor_msg = 10
                self.steering_val = self.error_lane * 0.004
            else:
                self.motor_msg = 20
                self.steering_val = self.error_lane * 0.004
    
    def publishCtrlCmd(self, motor_msg, servo_msg, brake_msg):
        self.ctrl_cmd_msg.velocity = motor_msg
        self.ctrl_cmd_msg.steering = self.steering_val
        self.ctrl_cmd_msg.brake = brake_msg
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

def nothing(x):
    pass


if __name__ == "__main__":
    control = Controller()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import sys,os
import rospy
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Bool, String, Int64MultiArray, Float64MultiArray, Int64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj, transform
from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, GPSMessage, CtrlCmd, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from utils import pathReader,findLocalPath,purePursuit
from std_msgs.msg import Int64

import tf
import time
from math import *

warnings.simplefilter(action='ignore', category=FutureWarning)

# 아이오닉 5 -> 조향값(servo_msg) 0일 때 직진 양수이면 좌회전 음수이면 우회전

DEFAULT_LFD = 18

class PurePursuit():
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        self.path_name = 'first_faster'

        # Publisher
        self.global_path_pub= rospy.Publisher('/global_path', Path, queue_size=1) ## global_path publisher 
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        # Subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB) ## Vehicle Status Subscriber 
        rospy.Subscriber("/imu", Imu, self.ImuCB) ## Vehicle Status Subscribe
        rospy.Subscriber("/traffic_light", Int64MultiArray, self.trafficCB)

        self.traffic_signal = 0

        self.green_count = 0
        self.red_count = 0

        self.steering_angle_to_servo_offset = 0.0 ## servo moter offset
        self.target_x = 0.0
        self.target_y = 0.0

        self.ctrl_cmd_msg = CtrlCmd()

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id='/odom'
        self.odom_msg.child_frame_id='/base_link1'
        self.odom_msg.pose.pose.position.x = 0.0
        self.odom_msg.pose.pose.position.y = 0.0
        
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0

        self.motor_msg = 0.0
        self.servo_msg = 0.0
        self.brake_msg = 0.0

        self.original_longitude = 0
        self.original_latitude = 0

        self.steering_offset = 0.05

        self.x, self.y = None, None
        self.br = tf.TransformBroadcaster()

        self.proj_UTM = Proj(proj='utm', zone = 52, elips='WGS84', preserve_units=False)
        
        ######## For Service ########
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.req_service = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)
        self.req = EventInfo()

        self.forward_mode()
        ################################

        # Class
        path_reader = pathReader('path_maker') ## 경로 파일의 위치
        self.pure_pursuit = purePursuit() ## purePursuit import
        
        # Read path
        self.global_path = path_reader.read_txt(self.path_name+".txt") ## 출력할 경로의 이름

        # Time var
        count = 0
        rate = rospy.Rate(20) 
                                           
        while not rospy.is_shutdown():
  
            self.global_path_pub.publish(self.global_path)

            local_path, current_waypoint = findLocalPath(self.global_path, self.odom_msg)

            self.convertLL2UTM()

            self.next_start_waypoint = current_waypoint
           
            self.pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
        
            self.pure_pursuit.getEgoStatus(self.odom_msg, self.motor_msg, False) 
    

            self.steering, self.target_x, self.target_y = self.pure_pursuit.steering_angle(DEFAULT_LFD)

            ############################ 일반 주행 조향값 및 속도 설정 ##################################

            # 조향 값 확인 : rostopic echo /sensors/s            # cv2.imshow("curve_slide_img", self.slide_img)
            # cv2.imshow("curve", self.curve_img)


            # cv2.imshow("original", cv2_image)ervo_position_command -> data
            # range : 0.0 ~ 1.0 (straight 0.5)

            self.ctrl_cmd_msg.longlCmdType = 1
            self.servo_msg = self.steering * self.steering_offset #+ self.steering_angle_to_servo_offset
            print(self.servo_msg)

            # servo_msg(조향각)에 따라서 엑셀 밟는 비율을 가변적으로 조정함.

            self.motor_msg = 0.1 # 0~1
            self.brake_msg = 0 # 0~1

            # echo red, green count
            print(f"green count: {self.green_count}\nred cound: {self.red_count}")

            ########################################################################################################################################################
            self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
            rate.sleep()
            ########################################################################################################################################################

###################################################################### Service Request  ######################################################################
    # option - 1 : ctrl_mode / 2 : gear / 4 : lamps / 6 : gear + lamps
    # gear - 1: P / 2 : R / 3 : N / 4 : D
##############################################################################################################################################################

    def forward_mode(self):
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 0
        self.req.lamps.emergencySignal = 0
        response = self.req_service(self.req)
        self.yaw_rear = False
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)

    def rear_mode(self):
        self.req.option = 2
        self.req.gear = 2
        response = self.req_service(self.req)
        self.yaw_rear = True
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
    
    def drive_left_signal(self):
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 1
        response = self.req_service(self.req)
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
    
    def drive_right_signal(self) :
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 2
        response = self.req_service(self.req)
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)

    def emergency_mode(self) :
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.emergencySignal = 1
        response = self.req_service(self.req)
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)

    def parking(self) :
        self.req.option = 6
        self.req.gear = 1
        self.req.lamps.turnSignal = 0
        response = self.req_service(self.req)
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)

    def brake(self) :
        self.ctrl_cmd_msg.longlCmdType = 1
        self.motor_msg = 0.0
        self.servo_msg = 0.0
        self.brake_msg = 1.0
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
    
###################################################################### Call Back ######################################################################

    def gpsCB(self, msg): 
        #UTMK
        self.original_longitude = msg.longitude
        self.original_latitude = msg.latitude

        self.lat = msg.latitude
        self.lon = msg.longitude
        
        self.convertLL2UTM()

        self.br.sendTransform((self.x, self.y, 0.),
                         tf.transformations.quaternion_from_euler(0,0,0.),
                         rospy.Time.now(),
                         "base_link",
                         "map")
        
        self.utm_msg = Float64MultiArray()

        self.utm_msg.data = [self.x, self.y]

        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.position.z = 0

    def convertLL2UTM(self) :
        xy_zone = self.proj_UTM(self.lon, self.lat)
        self.x, self.y = xy_zone[0], xy_zone[1]

    def ImuCB(self, msg) :
        self.odom_msg.pose.pose.orientation.x = msg.orientation.x
        self.odom_msg.pose.pose.orientation.y = msg.orientation.y
        self.odom_msg.pose.pose.orientation.z = msg.orientation.z
        self.odom_msg.pose.pose.orientation.w = msg.orientation.w

        quaternion = (self.odom_msg.pose.pose.orientation.x,self.odom_msg.pose.pose.orientation.y,self.odom_msg.pose.pose.orientation.z,self.odom_msg.pose.pose.orientation.w)
        _, _, self.yaw =  euler_from_quaternion(quaternion)
        self.yaw = degrees(self.yaw) 

    def trafficCB(self, msg):
        # Default : 0, Red : 1, Green :2
        self.green_count=msg.data[1]
        self.red_count=msg.data[0]

###################################################################### Function ######################################################################

    def publishCtrlCmd(self, motor_msg, servo_msg, brake_msg):
        self.ctrl_cmd_msg.accel = motor_msg
        self.ctrl_cmd_msg.steering = servo_msg
        self.ctrl_cmd_msg.brake = brake_msg
        # #print('pub', self.ctrl_cmd_msg.steering)
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def setMotorMsgWithVel(self, velocity):
        self.motor_msg = velocity

    def setServoMsgWithLfd(self, lfd):
        self.steering, self.target_x, self.target_y = self.pure_pursuit.steering_angle(lfd)
        self.servo_msg = self.steering*self.steering_offset

    def setBrakeMsgWithNum(self, brake):
        self.brake_msg = brake


if __name__ == '__main__':
    try:
        pure_pursuit_= PurePursuit()
    except rospy.ROSInterruptException:
        pass
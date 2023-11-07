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

import tf
import time
from math import *

warnings.simplefilter(action='ignore', category=FutureWarning)

# 아이오닉 5 -> 조향값(servo_msg) 0일 때 직진 양수이면 좌회전 음수이면 우회전
DEFAULT_LFD = 18

class PurePursuit():
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        # self.path_name = 'first_faster'
        self.path_name = 'second_faster'
        self.passed_curve = False

        # Publisher
        self.global_path_pub= rospy.Publisher('/global_path', Path, queue_size=1) ## global_path publisher 
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        # Subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB) ## Vehicle Status Subscriber 
        rospy.Subscriber("/imu", Imu, self.ImuCB) ## Vehicle Status Subscribe
        rospy.Subscriber("/traffic_light", Int64MultiArray, self.trafficCB)
        rospy.Subscriber("/gps_velocity", Float64, self.velocityCB)

        # traffic sign
        self.traffic_signal = 0

        self.green_count = 0
        self.red_count = 0

        # velocity -> accel로 오면서 항속을 하기 위해 만들어진 변수들
        self.gps_velocity = 0.0

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

        self.yaw = 0.0
        self.yaw_rear = False

        self.motor_msg = 0.0
        self.servo_msg = 0.0
        self.brake_msg = 0.0

        self.original_longitude = 0
        self.original_latitude = 0

        self.camera_servo_msg = 0.0
        self.camera_motor_msg = 10.0

        self.clear_stop_mission = False
        self.clear_start_mission = False

        self.dynamic_flag = False
        self.dynamic_done = False
        # self.dynamic_done = True

        self.curve_servo_msg = 0.0
        self.curve_motor_msg = 0.0

        self.Mission = "line_drive"

        self.green_count = 0
        self.red_count = 0

        self.dy_obs_info = [0, 0, 0, 0]

        self.T_mission = False

        # offset 0.05 -> 0.1 (pure pursuit 알고리즘 vehicle_length를 휠베이스 기반 3.000으로 바꾸고 나서 바꾼 값.)
        # self.steering_offset = 0.05
        self.steering_offset = 0.06

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
        rate = rospy.Rate(20) 
    
        while not rospy.is_shutdown():
  
            self.global_path_pub.publish(self.global_path)

            local_path, current_waypoint = findLocalPath(self.global_path, self.odom_msg)

            self.convertLL2UTM()

            self.next_start_waypoint = current_waypoint

            print("Current Waypoint: ", current_waypoint)
            print("red", self.red_count)
            print("green", self.green_count)
           
            self.pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
        
            self.pure_pursuit.getEgoStatus(self.odom_msg, self.motor_msg, False) 
    

            self.steering, self.target_x, self.target_y = self.pure_pursuit.steering_angle(DEFAULT_LFD)

            ############################ 일반 주행 조향값 및 속도 설정 ##################################

            # 조향 값 확인 : rostopic echo /sensors/s            # cv2.imshow("curve_slide_img", self.slide_img)
            # cv2.imshow("curve", self.curve_img)


            # cv2.imshow("original", cv2_image)ervo_position_command -> data
            # range : 0.0 ~ 1.0 (straight 0.5)

            self.ctrl_cmd_msg.longlCmdType = 2
            self.motor_msg = 19.75 # 0~1
            self.servo_msg = self.steering * self.steering_offset #+ self.steering_angle_to_servo_offset
            self.brake_msg = 0
            
            # gps - pure pursuit 기반 조향각 출력문
            # print(self.servo_msg)

            # servo_msg(조향각)에 따라서 엑셀 밟는 비율을 가변적으로 조정함.

            # 속도 조절 하는것. 악셀을 밟았다 뗐다가 하는 방향으로

            # # 속도 조절 알고리즘.
            # if self.gps_velocity < 20:
            #     self.motor_msg = 0.5
            #     self.brake_msg = 0
            # else:
            #     self.motor_msg = 0
            #     self.brake_msg = 0
            
            if self.path_name == 'first_faster':
            #---------------------------- 출발 -----------------------------------#
                if  current_waypoint <= 72:
                    self.setServoMsgWithLfd(8)
                    self.servo_msg /= 10
                    self.drive_left_signal()
                    continue
                elif current_waypoint <= 80  and self.clear_start_mission == False:
                    self.forward_mode()
                    self.clear_start_mission = True

            #---------------------------- 경사로 정지⋅ 출발 -----------------------------------#
                # 오르막 더 가속
                if 80 < current_waypoint <= 163: 
                    self.setMotorMsgWithVel(22)
                # 정지하는 코드
                elif 163 < current_waypoint <= 170 and self.clear_stop_mission == False:
                    self.brake()
                    self.clear_stop_mission = True
                    rospy.sleep(3.3) # 3.5초 동안 정지
                    continue
                # 재출발시 더 가속
                elif 170 < current_waypoint <= 180:
                    self.setMotorMsgWithVel(240)

                # 시간 재기 위한 테스트 코드
                # if current_waypoint >= 280:
                #     self.brake()
                #     rospy.sleep(3.3)
            # echo red, green count
            # print(f"green count: {self.green_count}\nred cound: {self.red_count}")


            #---------------------------- 곡선 코스 보정 -----------------------------------#
                if (529 < current_waypoint <= 560):
                    self.setMotorMsgWithVel(15)

                elif 1216 < current_waypoint <= 1256:
                    self.setMotorMsgWithVel(15)


            #---------------------------- 신호등 -----------------------------------#
            # 빨간 신호등 보면 멈출 위치 확인 & self.green_count - self.red_count 값 안전하게 400이하 -> 40으로 조정
                if 582 < current_waypoint <= 597 and  self.green_count - self.red_count < 500: # 첫번째 신호등
                    self.brake()
            
                if  1049 < current_waypoint <= 1057 and self.green_count - self.red_count < 500: # 두번째 신호등
                    self.brake()

            #---------------------------- T자 코스 -----------------------------------#
            if (self.path_name == 'parking_1' or 
                self.path_name == 'parking_2' or 
                self.path_name == 'parking_3'):

                self.setMotorMsgWithVel(9)
                # self.setServoMsgWithLfd(4)
                self.servo_msg *= 2
                
                self.T_mission = True
                if (self.path_name == 'parking_1' and current_waypoint + 10 >= len(self.global_path.poses)) : 
                    self.setMotorMsgWithVel(2)

                # 후진 조향각 40
                elif (self.path_name == 'parking_2' and current_waypoint <= 85) :
                    self.setMotorMsgWithVel(5)
                    self.servo_msg = 40
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue
                
                elif (self.path_name == 'parking_2' and  85 < current_waypoint < 116) : 
                    self.setMotorMsgWithVel(5)
                    self.servo_msg = 0
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue

                elif (self.path_name == 'parking_3' and current_waypoint <= 19) :
                    self.setMotorMsgWithVel(5)
                    # self.setServoMsgWithLfd(1)
                    self.servo_msg *= 2
                elif  (self.path_name == 'parking_3' and current_waypoint >= 20) :
                    self.setMotorMsgWithVel(12)
                    # self.setServoMsgWithLfd(5)
                    self.servo_msg *= 1.4
            else:
                self.T_mission = False

            if self.path_name == 'second_faster' and current_waypoint <= 25:
                self.setMotorMsgWithVel(15)
                # self.setServoMsgWithLfd(5)
                self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                continue

            # # 현재 후진상태이면 속도 2, 조향각 더줌.
            # if self.yaw_rear == True :
            #     self.setMotorMsgWithVel(2)
            #     # self.setServoMsgWithLfd(1)
            #     self.servo_msg *= 2
            #     if self.path_name == 'parking_3':
            #         self.setServoMsgWithLfd(18)
                

            # T자 주차를 위한 path switching

            if self.path_name == 'first_faster' and current_waypoint + 18  >= len(self.global_path.poses) :
                self.setServoMsgWithLfd(len(self.global_path.poses) - current_waypoint)
                
            # waypoint 5개는 안보겠다는 코드
            if current_waypoint + 5  >= len(self.global_path.poses) :
                if self.path_name == 'first_faster':
                    self.path_name = 'parking_1'
                    self.global_path = path_reader.read_txt(self.path_name+".txt")
                    # self.ctrl_cmd_msg.longlCmdType = 1
                    self.is_swith_path = False

                elif self.path_name == 'parking_1' and current_waypoint + 2 > len(self.global_path.poses): 
                    self.path_name = 'parking_2'
                    self.global_path = path_reader.read_txt(self.path_name+".txt")
                    self.brake()
                    self.is_swith_path = False
                    # 후진기어 넣기 전 멈추는 코드
                    for i in range(1000) :
                        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    rospy.sleep(1.2)
                    self.rear_mode()
                    continue

                elif self.path_name == 'parking_2' and current_waypoint +2 >= len(self.global_path.poses):
                    self.path_name = 'parking_3'
                    self.global_path = path_reader.read_txt(self.path_name+".txt")
                    self.brake()
                    rospy.sleep(1.2)
                    self.forward_mode()

                elif self.path_name == 'parking_3' and current_waypoint +5 >= len(self.global_path.poses): 
                    self.path_name = 'second_faster'
                    self.global_path = path_reader.read_txt(self.path_name+".txt")
                    self.is_swith_path = False
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue



            #---------------------------- 두번째 코스 시작 -----------------------------------#

            elif self.path_name == 'second_faster':

                #---------------------------- 신호등 -----------------------------------#
                # 빨간 신호등 보면 멈출 위치 확인 & self.green_count - self.red_count 값 안전하게 400이하 -> 40으로 조정
                if 33 < current_waypoint <= 41:
                    
                    self.drive_left_signal()
                    if (self.red_count - self.green_count > 150): # 첫번째 신호등
                        self.brake()
                elif 117 < current_waypoint <= 120:
                    self.forward_mode()


                #---------------------------- 가속 구간 -----------------------------------#
                if 415 < current_waypoint <= 505: # 481 -> 461
                    self.setMotorMsgWithVel(50)
                    self.setServoMsgWithLfd(25)
                    self.servo_msg /= 4
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue

                elif 505 < current_waypoint <= 530:
                    self.setMotorMsgWithVel(13)
                    self.setServoMsgWithLfd(25)
                    self.servo_msg /= 3
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue
                
                elif 530 < current_waypoint <= 545:
                    self.setMotorMsgWithVel(19.5)
                    self.setServoMsgWithLfd(25)
                    self.servo_msg /= 3
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue

                #---------------------------- 종료 미션 -----------------------------------#
                # 우측 방향지시등 점멸하고 통과하는 코드
                elif 700 < current_waypoint <= 817:
                    self.drive_right_signal()

                # 완전 종료 후 정차 코드
                elif 817 < current_waypoint <= 850:
                    print("echo")
                    self.motor_msg = 0
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    rospy.sleep(1)
                    self.parking()
                    continue
                    


                
                # else:
                #     self.setMotorMsgWithVel(19.5)
                #     self.setServoMsgWithLfd(20)
                #     self.setBrakeMsgWithNum(0.0)

            #     if 39 <= current_waypoint <= 107:
            #         self.setMotorMsgWithVel(15)
            #         self.setServoMsgWithLfd(4)

            #     elif 108 <= current_waypoint <= 121:
            #         self.setMotorMsgWithVel(15)
            #         self.setServoMsgWithLfd(4 + (current_waypoint - 107))

            #     elif 212 <= current_waypoint <= 275:
            #         self.setMotorMsgWithVel(15)
            #         self.setServoMsgWithLfd(4)

            #     elif 276 <= current_waypoint <= 289:
            #         self.setMotorMsgWithVel(15)
            #         self.setServoMsgWithLfd(4 + (current_waypoint - 275))

            #     elif 325 <= current_waypoint <= 380:
            #         self.setMotorMsgWithVel(15)
            #         self.setServoMsgWithLfd(4)
                
            #     elif 381 <= current_waypoint <= 384:
            #         self.setMotorMsgWithVel(15)
            #         self.setServoMsgWithLfd(4 + (current_waypoint - 380))
                    
            #     elif 643 <= current_waypoint <= 707 :
            #         self.setMotorMsgWithVel(15)
            #         self.setServoMsgWithLfd(4)
                
            #     elif 708 <= current_waypoint <= 721 :
            #         self.setMotorMsgWithVel(15)
            #         self.setServoMsgWithLfd(4 + (current_waypoint - 707))
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
    #     self.gps_velocity = msg.
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


    def velocityCB(self, msg):
        self.gps_velocity = msg.data

###################################################################### Function ######################################################################

    def publishCtrlCmd(self, motor_msg, servo_msg, brake_msg):
        self.ctrl_cmd_msg.velocity = motor_msg
        self.ctrl_cmd_msg.steering = servo_msg
        self.ctrl_cmd_msg.brake = brake_msg
        # #print('pub', self.ctrl_cmd_msg.steering)
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    # 기존의 velocity 값으로 속도를 조절하던 코드
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
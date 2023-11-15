#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import warnings
import rospy
import tf
import time
from math import *
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64, Int64MultiArray, Float64MultiArray, Int64, Int16MultiArray
from sensor_msgs.msg import Imu, CompressedImage
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from pyproj import Proj, transform
from morai_msgs.msg import GPSMessage, CtrlCmd, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from defined_func import pathLoader,findLocalPath,purePursuit
from object_detector.msg import ObjectInfo


warnings.simplefilter(action='ignore', category=FutureWarning)

DEFAULT_LFD = 18
#양수 좌회전 음수 우회전

class PurePursuit():
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        self.path_name = 'before_parking'
        # self.path_name = 'after_parking'
        
# 
        # Publisher
        self.global_path_pub= rospy.Publisher('/global_path', Path, queue_size=1) ## global_path publisher 
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        # Subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCallback) ## Vehicle Status Subscriber 
        rospy.Subscriber("/imu", Imu, self.ImuCallback) ## Vehicle Status Subscribe
        rospy.Subscriber("/traffic_light", Int64MultiArray, self.trafficCallback)
        rospy.Subscriber("/gps_velocity", Float64, self.velocityCallback)
        rospy.Subscriber("/s_ctrl_cmd", CtrlCmd, self.sMissionCallback) # Drive for camera
        rospy.Subscriber("/z_ctrl_cmd", CtrlCmd, self.zMissionCallback)
        rospy.Subscriber("/z_curve", Int16MultiArray, self.zCurveCallback)
        rospy.Subscriber("/bounding_box", MarkerArray, self.ObstacleCallback)
        rospy.Subscriber("/object_info", ObjectInfo, self.obsCountCallback)

        # wait for message
        # rospy.wait_for_message("/image_jpeg/compressed", CompressedImage)
        # rospy.wait_for_message("/gps", GPSMessage)
        # rospy.wait_for_message("/imu", Imu)

        # service
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.req_service = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)
        self.req = EventInfo()

        self.obs = []
        self.s_flag = False
        self.start_s_flag = 0

        # 신호등
        self.traffic_signal = 0

        self.green_count = 0
        self.red_count = 0

        # velocity -> accel로 오면서 항속을 하기 위해 만들어진 변수들
        # self.gps_velocity = 0.0

        self.steering_angle_to_servo_offset = 0.0 
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

        self.sliding_steering_msg = 0.0
        self.sliding_motor_msg = 10.0

        self.z_sliding_steering_msg = 0.0
        self.z_sliding_motor_msg = 10.0

        self.first_curve = 0
        self.second_curve = 0

        self.z_first = 0
        self.z_second = 0

        self.clear_stop_mission = False
        self.clear_start_mission = False


        self.curve_servo_msg = 0.0
        self.curve_motor_msg = 0.0

        # self.Mission = "line_drive"

        self.green_count = 0
        self.red_count = 0

        self.dy_obs_mission = False
        self.dy_obs_orient_y = 0

        # callback 받는 거
        self.obs_count = 0        

        self.T_mission = False

        self.steering_offset = 0.06

        self.x, self.y = None, None
        self.br = tf.TransformBroadcaster()

        self.proj_UTM = Proj(proj='utm', zone = 52, elips='WGS84', preserve_units=False)
        
        

        self.D_mode()

        path_load = pathLoader('path_maker') ## 경로 파일의 위치
        self.pure_pursuit = purePursuit() ## purePursuit import
        
        self.global_path = path_load.load_txt(self.path_name+".txt") ## 출력할 경로의 이름

        rate = rospy.Rate(20) 
    
        while not rospy.is_shutdown():
  
            self.global_path_pub.publish(self.global_path)

            local_path, current_waypoint = findLocalPath(self.global_path, self.odom_msg)

            self.convertLL2UTM()

            self.next_start_waypoint = current_waypoint

            # print("Current Waypoint: ", current_waypoint)
            # print("red_traffic", self.red_count)
            # print("green_traffic", self.green_count)
           
            self.pure_pursuit.getPath(local_path) 
        
            self.pure_pursuit.getEgoStatus(self.odom_msg, self.motor_msg, False) 
    

            self.steering, self.target_x, self.target_y = self.pure_pursuit.steering_angle(DEFAULT_LFD)

            #-------------------------------------- 초기값 설정 ----------------------------------------#


            self.ctrl_cmd_msg.longlCmdType = 2
            self.motor_msg = 19.75 # 0~1
            self.servo_msg = self.steering * self.steering_offset 
            self.brake_msg = 0

            
            
            if self.path_name == 'before_parking':
            #---------------------------- 출발 -----------------------------------#
                if  current_waypoint <= 72:
                    self.setSteering(8)
                    self.servo_msg /= 10
                    self.left_signal()
                    continue
                elif current_waypoint <= 80  and self.clear_start_mission == False:
                    self.D_mode()
                    self.clear_start_mission = True

            #---------------------------- 경사로 정지⋅ 출발 -----------------------------------#
                # 오르막 더 가속
                if 80 < current_waypoint <= 163: 
                    self.setVelocity(21)

                # 정지하는 코드
                elif 163 < current_waypoint <= 170 and self.clear_stop_mission == False:
                    self.brake()
                    self.clear_stop_mission = True
                    rospy.sleep(3.3) # 3.3초 동안 정지
                    continue

                # 재출발시 더 가속
                elif 170 < current_waypoint <= 180:
                    self.setVelocity(19.75)
            
            #---------------------------- 직각 코스 -----------------------------------#
                if (320 < current_waypoint <= 374):
                    self.setVelocity(15)

                if self.original_latitude <= 1.0 and self.original_longitude <= 1.0 and self.s_flag == False:
                    if self.first_curve == 1 and self.second_curve == 0 and self.z_first == 0:
                        sec = time.time()
                        while time.time() - sec <= 4:
                            self.publishCtrlCmd(10, 40, self.brake_msg)
                        self.z_first = 1
                    elif self.first_curve == 1 and self.second_curve == 1 and self.z_second == 0:
                        sec = time.time()
                        while time.time() - sec <= 4:
                            self.publishCtrlCmd(10, -40, self.brake_msg)
                        self.z_second = 1

                    else:
                        if self.z_second == 1:
                            self.publishCtrlCmd(19.75, self.z_sliding_steering_msg, self.brake_msg)
                        else:
                            self.publishCtrlCmd(3, self.z_sliding_steering_msg, self.brake_msg)
                    continue

                if 490 < current_waypoint <= 510:
                    self.s_flag = True
                    
            #---------------------------- 곡선 코스 보정 -----------------------------------#
                # if (521 < current_waypoint <= 552):
                #     self.setVelocity(17)

                if 1208 < current_waypoint <= 1248:
                    self.setVelocity(17)

                if 1300 < current_waypoint <= 1341:
                    self.setVelocity(15)

            #---------------------------- 신호등 -----------------------------------#
                if 574 < current_waypoint <= 589 and  self.green_count - self.red_count < 500: # 첫번째 신호등s
                    self.brake()
            
                if  1041 < current_waypoint <= 1049 and self.green_count - self.red_count < 500: # 두번째 신호등
                    self.brake()
                
            #---------------------------- s자 -----------------------------------#
                if 752 < current_waypoint <= 772 and self.s_flag:
                    self.motor_msg = 15
                    
                if 772 < current_waypoint <= 781 and self.s_flag:
                    # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    self.servo_msg = -40
                    self.motor_msg = 10
                    
                    
                if self.original_latitude <= 1.0 and self.original_longitude <= 1.0 and self.s_flag: # GPS 음영구역 진입 시 Camera Steering으로 주행
                    if self.start_s_flag == 0:
                        sec_s = time.time()
                        while time.time() - sec_s <= 0.3:
                            # print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                            self.publishCtrlCmd(10, -5, self.brake_msg)
                        self.start_s_flag = 1
                    else:
                        # print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
                        self.publishCtrlCmd(self.sliding_motor_msg, self.sliding_steering_msg, self.brake_msg)
                        continue
            
            #---------------------------- 동적 장애물 -----------------------------------#

                if ((992 <= current_waypoint <= 1032 or 1182 <= current_waypoint <= 1212 or 1342 <= current_waypoint <= 1392) and not self.dy_obs_mission):
                    #장애물이 있는가?
                    if(len(self.obs) > 0):
                        self.dy_obs_orient_y = self.obs[0][1]
                        print(self.dy_obs_orient_y)
                        while(True):
                            self.brake()
                            self.emergency_mode()
                            self.dy_obs_mission = True
                            if len(self.obs) == 0 or abs(self.dy_obs_orient_y - self.obs[0][1]) >= 3.6:
                                self.D_mode()
                                break
                        continue

                

            #---------------------------- T자 코스 -----------------------------------#
            if (self.path_name == 'parking_1' or 
                self.path_name == 'parking_2' or 
                self.path_name == 'parking_3'):

                self.setVelocity(9)
                self.servo_msg *= 2
                
                self.T_mission = True
                if (self.path_name == 'parking_1' and current_waypoint + 10 >= len(self.global_path.poses)) : 
                    self.setVelocity(2)

                # 후진 조향각 40
                elif (self.path_name == 'parking_2' and current_waypoint <= 85) :
                    self.setVelocity(5)
                    self.servo_msg = 40
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue
                
                elif (self.path_name == 'parking_2' and  85 < current_waypoint < 116) : 
                    self.setVelocity(5)
                    self.servo_msg = 0
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue

                elif (self.path_name == 'parking_3' and current_waypoint <= 19) :
                    self.setVelocity(5)
                    self.servo_msg *= 2
                elif  (self.path_name == 'parking_3' and current_waypoint >= 20) :
                    self.setVelocity(12)
                    self.servo_msg *= 1.4
            else:
                self.T_mission = False

            if self.path_name == 'after_parking' and current_waypoint <= 25:
                self.setVelocity(15)
                self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                continue
                

            # T자 주차를 위한 path switching

            if self.path_name == 'before_parking' and current_waypoint + 18  >= len(self.global_path.poses) :
                self.setSteering(len(self.global_path.poses) - current_waypoint)

                
            # waypoint 5개는 안보겠다는 코드
            if current_waypoint + 5  >= len(self.global_path.poses) :
                if self.path_name == 'before_parking':
                    self.path_name = 'parking_1'
                    self.global_path = path_load.load_txt(self.path_name+".txt")

                elif self.path_name == 'parking_1' and current_waypoint + 2 > len(self.global_path.poses): 
                    self.path_name = 'parking_2'
                    self.global_path = path_load.load_txt(self.path_name+".txt")
                    self.brake()
                    # 후진기어 넣기 전 멈추는 코드
                    for i in range(1000) :
                        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    rospy.sleep(1.2)
                    self.R_mode()
                    continue

                elif self.path_name == 'parking_2' and current_waypoint +2 >= len(self.global_path.poses):
                    self.path_name = 'parking_3'
                    self.global_path = path_load.load_txt(self.path_name+".txt")
                    self.brake()
                    rospy.sleep(1.2)
                    self.D_mode()

                elif self.path_name == 'parking_3' and current_waypoint +5 >= len(self.global_path.poses): 
                    self.path_name = 'after_parking'
                    self.global_path = path_load.load_txt(self.path_name+".txt")
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue



            #---------------------------- 두번째 코스 시작 -----------------------------------#

            elif self.path_name == 'after_parking':

                #---------------------------- 신호등 -----------------------------------#
                # 빨간 신호등 보면 멈출 위치 확인 & self.green_count - self.red_count 값 안전하게 400이하 -> 40으로 조정
                if 33 < current_waypoint <= 41:
                    
                    self.left_signal()
                    if (self.red_count - self.green_count > 150): # 첫번째 신호등
                        self.brake()
                elif 117 < current_waypoint <= 120:
                    self.D_mode()


                #---------------------------- 가속 구간 -----------------------------------#
                if 415 < current_waypoint <= 505: # 481 -> 461
                    self.setVelocity(40)
                    self.setSteering(25)
                    self.servo_msg /= 4
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue

                elif 505 < current_waypoint <= 530:
                    self.setVelocity(13)
                    self.setSteering(25)
                    self.servo_msg /= 3
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue
                
                elif 530 < current_waypoint <= 545:
                    self.setVelocity(19.5)
                    self.setSteering(25)
                    self.servo_msg /= 3
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    continue

                #--------------------------------after_parking _ 장애물 미션--------------------------------#
                if (101 <= current_waypoint <= 160 or 585 <= current_waypoint <= 620) and self.dy_obs_mission == False:
                    if(len(self.obs) > 0):
                        self.dy_obs_orient_y = self.obs[0][1]
                        print(self.dy_obs_orient_y)
                        while(True):
                            self.brake()
                            self.emergency_mode()
                            self.dy_obs_mission = True
                            if len(self.obs) == 0 or abs(self.dy_obs_orient_y - self.obs[0][1]) >= 3.6:
                                self.D_mode()
                                break
                        continue

                # if self.dy_obs_mission and (160 < current_waypoint <= 170 or 643 < current_waypoint <= 653):
                #     self.D_mode()

                #---------------------------- 종료 미션 -----------------------------------#
                # 우측 방향지시등 점멸하고 통과하는 코드
                elif 700 < current_waypoint <= 817:
                    self.right_signal()

                # 완전 종료 후 정차 코드
                elif 817 < current_waypoint <= 850:
                    # print("echo")
                    self.motor_msg = 0
                    self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
                    rospy.sleep(1)
                    self.parking()
                    continue


                    
            self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
            rate.sleep()

#---------------------------------------------------------------------기어정보-----------------------------------------------------------------------#
    # 추가사항 - 1 : 주행설정 / 2 : 기어 / 4 : 등 / 6 : 기어 + 등
    # 기어 - 1: 주차 / 2 : 후진 / 3 : 중립 / 4 : 주행
#--------------------------------------------------------------------------------------------------------------------------------------------------#
    
    def D_mode(self):
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 0
        self.req.lamps.emergencySignal = 0
        response = self.req_service(self.req)
        self.yaw_rear = False
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)

    def R_mode(self):
        self.req.option = 2
        self.req.gear = 2
        response = self.req_service(self.req)
        self.yaw_rear = True
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
    
    def left_signal(self):
        self.req.option = 6
        self.req.gear = 4
        self.req.lamps.turnSignal = 1
        response = self.req_service(self.req)
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
    
    def right_signal(self) :
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


    def brake(self) :
        self.motor_msg = 0.0
        self.servo_msg = 0.0
        self.brake_msg = 0.0
        self.publishCtrlCmd(self.motor_msg, self.servo_msg, self.brake_msg)
    
#----------------------------------------------------------------------------------콜백 함수들 --------------------------------------------------------------------#
    def sMissionCallback(self, msg) : 
        self.sliding_steering_msg = msg.steering
        self.sliding_motor_msg = msg.velocity

    def zMissionCallback(self, msg) : 
        self.z_sliding_steering_msg = msg.steering
        self.z_sliding_motor_msg = msg.velocity

    def zCurveCallback(self, msg):
        self.first_curve = msg.data[0]
        self.second_curve = msg.data[1]

        # print(self.first_curve, self.second_curve)

    def gpsCallback(self, msg): 
        try:
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
        except:
            pass


    def convertLL2UTM(self) :
        xy_zone = self.proj_UTM(self.lon, self.lat)
        self.x, self.y = xy_zone[0], xy_zone[1]

    def ImuCallback(self, msg) :
        try:
            self.odom_msg.pose.pose.orientation.x = msg.orientation.x
            self.odom_msg.pose.pose.orientation.y = msg.orientation.y
            self.odom_msg.pose.pose.orientation.z = msg.orientation.z
            self.odom_msg.pose.pose.orientation.w = msg.orientation.w

            quaternion = (self.odom_msg.pose.pose.orientation.x,self.odom_msg.pose.pose.orientation.y,self.odom_msg.pose.pose.orientation.z,self.odom_msg.pose.pose.orientation.w)
            _, _, self.yaw =  euler_from_quaternion(quaternion)
            self.yaw = degrees(self.yaw)
        except:
            pass 

    def trafficCallback(self, msg):
        # Default : 0, Red : 1, Green :2
        self.green_count=msg.data[1]
        self.red_count=msg.data[0]


    def velocityCallback(self, msg):
        self.gps_velocity = msg.data
    
    # 
    def ObstacleCallback(self, msg):
        # 장애물 없을경우 예외처리 size() > 0 이런식
        if len(msg.markers) > 0 and self.obs_count > 0:
            obs = []
            for i in range(len(msg.markers)):
                if(0 < msg.markers[i].pose.position.x <= 7.0 and abs(msg.markers[i].pose.position.y) <= 2.6):
                    obs.append([(msg.markers[i].pose.position.x, msg.markers[i].pose.position.y)])
            if(len(obs) > 0):
                self.obs = sorted(obs, key = lambda x: obs[0])[0]
            # print(self.obs)
            
        else:
            self.obs = []

    # 
    def obsCountCallback(self, msg):
        self.obs_count = msg.objectCounts
        
        
        # print(self.obs_count)

        
            
    
###################################################################### Function ######################################################################

    def publishCtrlCmd(self, motor_msg, servo_msg, brake_msg):
        self.ctrl_cmd_msg.velocity = motor_msg
        self.ctrl_cmd_msg.steering = servo_msg
        self.ctrl_cmd_msg.brake = brake_msg
        # #print('pub', self.ctrl_cmd_msg.steering)
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    # 기존의 velocity 값으로 속도를 조절하던 코드
    def setVelocity(self, velocity):
        self.motor_msg = velocity

    def setSteering(self, lfd):
        self.steering, self.target_x, self.target_y = self.pure_pursuit.steering_angle(lfd)
        self.servo_msg = self.steering*self.steering_offset

    def setBrakeMsgWithNum(self, brake):
        self.brake_msg = brake


if __name__ == '__main__':
    try:
        pure_pursuit_= PurePursuit()
    except rospy.ROSInterruptException:
        pass
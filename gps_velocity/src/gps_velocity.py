#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import os
import sys
import rospy
import rospkg
from pyproj import Proj, transform
from sensor_msgs.msg import NavSatFix
from morai_msgs.msg  import EgoVehicleStatus, GPSMessage
from math import pi, sqrt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float64
import tf


warnings.simplefilter(action='ignore', category=FutureWarning)

class GpsVelocity :
    def __init__(self):
        rospy.init_node('gps_velocity', anonymous=True)
        
        self.longitude = 0
        self.latitude = 0
        self.altitude = 0

        gps_velocity_pub = rospy.Publisher('/gps_velocity', Float64, queue_size=1)
        rospy.Subscriber("/gps",GPSMessage, self.gps_callback)

        self.prev_longitude = 0
        self.prev_latitude = 0


        # 
        # self.threshold_ratio = 0.1
        # gps 통해 구하는 속도의 튀는값 보정 위한 
        self.threshold_num = 6

        self.prev_velocity = Float64()
        self.prev_velocity = 0.0
        self.velocity = Float64()
        self.velocity = 0.0

        self.utmk_coordinate = Point() 

        self.proj_UTM = Proj(proj='utm', zone = 52, elips='WGS84', preserve_units=False)


        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            self.get_velocity()
            gps_velocity_pub.publish(self.velocity)
            rate.sleep()    


    def get_velocity(self):
        
        xy_zone = self.proj_UTM(self.longitude, self.latitude)
        self.x, self.y = xy_zone[0], xy_zone[1]

        self.utmk_coordinate.x = self.x
        self.utmk_coordinate.y = self.y
        self.utmk_coordinate.z = 0

        distance = sqrt(pow(self.utmk_coordinate.x - self.prev_longitude, 2) + pow(self.utmk_coordinate.y - self.prev_latitude, 2))
        self.velocity = distance / 0.1 * 3.6
        
        # # 이전값에 일정비율을 곱해서 더한것. 튀는값 threshold
        # self.a = self.prev_velocity + self.prev_velocity * self.threshold_ratio
        # if self.a < self.velocity:
        #     print(self.a)
        #     self.velocity = self.prev_velocity


        # 새로 구한 값(self.velocity)과 이전값 (prev_velocity)의 차이가 일정 범위 이상이면 이전값으로 유지. 
        # 현재는 악셀 0.1 이기 때문에 추후 값 조정 필요.
        if abs(self.velocity - self.prev_velocity) > self.threshold_num:
            self.velocity = self.prev_velocity

        # prev 값들 update
        self.prev_longitude = self.utmk_coordinate.x
        self.prev_latitude = self.utmk_coordinate.y
        self.prev_velocity = self.velocity

    def gps_callback(self, msg): 
        self.longitude = msg.longitude
        self.latitude = msg.latitude
        self.altitude = msg.altitude

    # ratio = 0.3
    #   14 <- 20 -> 26
    #    7 <- 10 -> 13

if __name__ == '__main__':
    try:
        gps_velocity_ = GpsVelocity()
    except rospy.ROSInterruptException:
        pass


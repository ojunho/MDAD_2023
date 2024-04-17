#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospkg
from tf.transformations import euler_from_quaternion
from math import sin, cos, sqrt, pow, atan2, pi
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import *

class purePursuit :
    def __init__(self):
        self.forward_point=Point()
        self.current_position=Point()
        self.forward_flag=False
        self.lfd = 1
        self.min_lfd = 13
        self.max_lfd = 20
        
        self.vehicle_length = 3.000
        self.steering = 0
        
    def getPath(self,msg):
        self.path = msg
    
    def getEgoStatus(self, odom_msg, vehicle_vel, rear):

        quaternion = (odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w)
        _, _, vehicle_yaw = euler_from_quaternion(quaternion)

        if rear == True :
            vehicle_yaw += 180

        self.vehicle_yaw = vehicle_yaw
        self.current_vel = vehicle_vel

        self.current_position.x = odom_msg.pose.pose.position.x
        self.current_position.y = odom_msg.pose.pose.position.y
        self.current_position.z = 0.0

    def steering_angle(self, lfd,):
        vehicle_position = self.current_position
        rotated_point = Point()
        
        self.forward_flag = False

        self.lfd = 1

        for i in self.path.poses :
            path_point = i.pose.position
            
            dx = path_point.x - vehicle_position.x
            dy = path_point.y - vehicle_position.y
            
            rotated_point.x = cos(self.vehicle_yaw)*dx + sin(self.vehicle_yaw)*dy
            rotated_point.y = sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy
            
            
            if rotated_point.x > 0 :
                distance = sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))


                if distance >= self.lfd :
                    self.lfd = lfd
                    self.forward_point = path_point
                    self.forward_flag=True
                    break
             
        theta = atan2(rotated_point.y,rotated_point.x)

        self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi*(-1)

        return self.steering, path_point.x, path_point.y
    
    def rear_steering_angle(self):
        vehicle_position = self.current_position
        rotated_point = Point()
        self.forward_flag = False
        

        self.lfd = 1
        self.min_lfd = 1
        self.max_lfd = 2

        for i in self.path.poses :
            path_point = i.pose.position
            dx = path_point.x - vehicle_position.x
            dy = path_point.y - vehicle_position.y
            rotated_point.x = cos(self.vehicle_yaw)*dx + sin(self.vehicle_yaw)*dy
            rotated_point.y = sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy

            if rotated_point.x > 0 :
                distance = sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))
                
                if distance >= self.lfd :
                    self.lfd = self.current_vel / 1.8
                    if self.lfd < self.min_lfd : 
                        self.lfd=self.min_lfd
                    elif self.lfd > self.max_lfd :
                        self.lfd=self.max_lfd
                    self.forward_point=path_point
                    self.forward_flag=True
                    break

        theta = atan2(rotated_point.y,rotated_point.x)
        if self.forward_flag :
            self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi*(-1)
            return self.steering, path_point.x, path_point.y
        else : 
            self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi*(-1)
            return self.steering, path_point.x, path_point.y


class pathLoader:
    def __init__(self,package_name):
        rospack = rospkg.RosPack()
        self.pack_path = rospack.get_path(package_name)

    def load_txt(self,file_name):
        full_file_name = self.pack_path + "/path/" + file_name
        openFile = open(full_file_name, 'r')
        target_path=Path()
        
        target_path.header.frame_id='map'
        line = openFile.readlines()
        for i in line :
            tmp = i.split()
            read_pose = PoseStamped()
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.z = float(tmp[2])
            read_pose.pose.orientation.x = 0
            read_pose.pose.orientation.y = 0
            read_pose.pose.orientation.z = 0
            read_pose.pose.orientation.w = 1
            target_path.poses.append(read_pose)
        
        openFile.close()
        return target_path
      

def findLocalPath(global_path, odom_msg):
    target_path =  Path()
    current_x = odom_msg.pose.pose.position.x
    current_y = odom_msg.pose.pose.position.y
    current_waypoint = 0
    minimum_distance = float('inf')

    for i in range(len(global_path.poses)) :
        dx = current_x - global_path.poses[i].pose.position.x
        dy = current_y - global_path.poses[i].pose.position.y
        distance = sqrt(dx*dx + dy*dy)

        if distance < minimum_distance :
            minimum_distance = distance
            current_waypoint = i

    if current_waypoint + 100 > len(global_path.poses) : 
        last_local_waypoint = len(global_path.poses)
    else :
        last_local_waypoint = current_waypoint + 100

    target_path.header.frame_id = 'map'

    for i in range(current_waypoint,last_local_waypoint) :
        tmp = PoseStamped()
        tmp.pose.position.x = global_path.poses[i].pose.position.x
        tmp.pose.position.y = global_path.poses[i].pose.position.y
        tmp.pose.position.z = global_path.poses[i].pose.position.z
        tmp.pose.orientation.x = 0
        tmp.pose.orientation.y = 0
        tmp.pose.orientation.z = 0
        tmp.pose.orientation.w = 1
        target_path.poses.append(tmp)

    return target_path, current_waypoint 



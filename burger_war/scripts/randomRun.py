#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is rumdom run node.
subscribe 'scan' topcs.
Publish 'cmd_vel' topic. 
mainly use for simple sample program

by Takuya Yamaguhi.
'''

import rospy
import random
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from aruco_msgs.msg import MarkerArray

class RandomBot():
    def __init__(self, bot_name="NoName"):
        # bot name 
        self.name = bot_name

        #wall distance
        self.near_wall_range = 0.2  # [m]
        # speed [m/s]
        self.speed = 0.1
        # lidar scan
        self.scan = []
        # target_id
        self.id_list = np.zeros(1)

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        #subscriber
        self.lidar_sub = rospy.Subscriber('scan',LaserScan,self.LidarCallback)
        self.id_sub = rospy.Subscriber('target_id',MarkerArray,self.IdCallback)

        self.twist = Twist()
        self.twist.linear.x = self.speed; self.twist.linear.y = 0.; self.twist.linear.z = 0.
        self.twist.angular.x = 0.; self.twist.angular.y = 0.; self.twist.angular.z = 0.
    
    def LidarCallback(self,data):
        scan = data.ranges
        self.scan = scan
        near_wall = self.NearWall(scan)
        if near_wall:
            self.twist.linear.x = -self.speed / 2
        else:
            self.twist.linear.x = self.speed

    def NearWall(self, scan):
        if not len(scan) == 360:
            return False
        forword_scan = scan[:15] + scan[-15:]
        # drop too small value ex) 0.0
        forword_scan = [x for x in forword_scan if x > 0.1]
        if min(forword_scan) < 0.2:
            return True
        return False
    
    def IdCallback(self, data):
        id = data.markers[0].id
        self.id_list = np.append(self.id_list, id)

    def calcTwist(self):
        value = random.randint(1,1000)
        if value < 350:
            x = 0.2
            th = 0
        elif value < 500:
            x = -0.2
            th = 0
        elif value < 750:
            x = 0
            th = 0.3
        elif value < 1000:
            x = 0
            th = -0.3
        else:
            x = 0
            th = 0
        self.twist = Twist()
        self.twist.linear.x = x; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = th
        return self.twist

    def FirstPoint(self):
        self.twist = Twist()
        self.twist.linear.x = 0.3; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
        return self.twist

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        while not rospy.is_shutdown():
            if self.id_list[-1]==0:
                twist_first = self.FirstPoint()
                print(twist_first)
                self.vel_pub.publish(twist_first)
            else:
                twist = self.calcTwist()
                print(twist)
                print(self.id_list)
                self.vel_pub.publish(twist)
            
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random')
    bot.strategy()


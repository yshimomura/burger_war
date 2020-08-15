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
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from aruco_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
import actionlib #topic通信においてクライアントからの返信を要求するライブラリ
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

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
        # position
        self.pose_x = 0
        self.pose_y = 0
        self.th = 0

        # publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # subscriber
        """self.lidar_sub = rospy.Subscriber('scan',LaserScan,self.LidarCallback)"""
        self.id_sub = rospy.Subscriber('target_id',MarkerArray,self.IdCallback)
        self.pose_sub = rospy.Subscriber('odom', Odometry, self.poseCallback)
        #actionlib
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        self.twist = Twist()
        self.twist.linear.x = self.speed; self.twist.linear.y = 0.; self.twist.linear.z = 0.
        self.twist.angular.x = 0.; self.twist.angular.y = 0.; self.twist.angular.z = 0.
    """
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
        """
    def IdCallback(self, data):
        id = data.markers[0].id
        if self.id_list[-1]!=id:
            self.id_list = np.append(self.id_list, id)
        else:
            pass

    def poseCallback(self, data):
        '''
        pose topic from amcl localizer
        update robot twist
        '''
        pose_x = data.pose.pose.position.x
        self.pose_x = pose_x
        pose_y = data.pose.pose.position.y
        self.pose_y = pose_y
        quaternion = data.pose.pose.orientation
        rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        th = rpy[2]
        self.th = th
        """
        th_xy = self.calcTargetTheta(pose_x,pose_y)
        
        th_diff = th_xy - th
        while not PI >= th_diff >= -PI:
            if th_diff > 0:
                th_diff -= 2*PI
            elif th_diff < 0:
                th_diff += 2*PI

        delta_th = self.calcDeltaTheta(th_diff)
        new_twist_ang_z = max(-0.3, min((th_diff + delta_th) * self.k , 0.3))
        
        self.twist.angular.z = new_twist_ang_z
        """
        print(self.pose_x,self.pose_y,self.th)

    def calcTwist(self):
        value = random.randint(1,1000)
        if value < 300:
            x = 0.3
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
    
    def SecondPoint(self):
        self.twist = Twist()
        if self.pose_y>-0.7:
            self.twist.linear.x = -0.3; self.twist.linear.y = 0; self.twist.linear.z = 0
            self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
        elif self.th>1:
            self.twist.linear.x = 0; self.twist.linear.y = 0; self.twist.linear.z = 0
            self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0.5
        else:
            self.calcTwist()
        return self.twist

    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

    def strategy(self):
        r = rospy.Rate(3) # change speed 1fps

        self.setGoal(-0.5,0,0)
        self.setGoal(-0.5,0,3.1415/2)
        self.setGoal(0,0.5,0)
        self.setGoal(0,0.5,3.1415)
        self.setGoal(-0.5,0,-3.1415/2)
        self.setGoal(0,-0.5,0)
        self.setGoal(0,-0.5,3.1415)
        """
        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        while not rospy.is_shutdown():
            if self.id_list[-1]==0:
                twist_first = self.FirstPoint()
                print(twist_first)
                self.vel_pub.publish(twist_first)
            elif self.id_list[-1]==2:
                twist_second = self.SecondPoint()
                print(twist_second)
                self.vel_pub.publish(twist_second)
            else:
                twist = self.calcTwist()
                print(twist)
                print(self.id_list)
                self.vel_pub.publish(twist)
            r.sleep()
            """

if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random')
    bot.strategy()


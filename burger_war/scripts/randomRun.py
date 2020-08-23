#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random
import numpy as np
import math
import tf
import cv2
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from aruco_msgs.msg import MarkerArray
from cv_bridge import CvBridge, CvBridgeError
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
        # lidar scan
        self.scan = []
        # target_id
        self.id_list = np.zeros(1)
        # position
        self.pose_x = 0
        self.pose_y = 0
        self.th = 0
        # speed [m/s]
        self.speed = 0.1

        self.bridge = CvBridge()

        # publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # subscriber
        self.lidar_sub = rospy.Subscriber('scan',LaserScan,self.LidarCallback)
        self.id_sub = rospy.Subscriber('target_id',MarkerArray,self.IdCallback)
        self.pose_sub = rospy.Subscriber('odom', Odometry, self.PoseCallback)
        self.image_sub = rospy.Subscriber('image_raw', Image, self.ImageCallback)
        #actionlib
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        self.twist = Twist()
        self.twist.linear.x = 0; self.twist.linear.y = 0.; self.twist.linear.z = 0.
        self.twist.angular.x = 0.; self.twist.angular.y = 0.; self.twist.angular.z = 0.
        #self.enemy_detector = EnemyDetector()

        self.image_state = 0
        
    def LidarCallback(self, data):
        scan = data.ranges
        self.scan = scan
        self.near_wall = self.NearWall(scan)

    def NearWall(self, scan):
        if not len(scan) == 360:
            return False
        forword_scan = scan[:20] + scan[-20:]
        back_scan = scan[160:200]
        # drop too small value ex) 0.0
        forword_scan = [x for x in forword_scan if x > 0.1]
        back_scan = [x for x in back_scan if x > 0.1]
        if min(forword_scan) < self.near_wall_range:
            return 1
        elif min(back_scan) < self.near_wall_range:
            return 2
        return False

    def IdCallback(self, data):
        id = data.markers[0].id
        if self.id_list[-1]!=id:
            self.id_list = np.append(self.id_list, id)
        else:
            pass

    def PoseCallback(self, data):
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
        print(self.pose_x,self.pose_y,self.th)

    def ImageCallback(self, image):
        try:
            frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError, e:
            print e
        input_image = np.array(frame, dtype=np.uint8)

        rects = self.process_image(input_image)
        print(rects)
        if rects!=[]:
            if rects[0][0] + rects[0][2]/2 < 220:
                self.image_state = 3
            elif rects[0][0] + rects[0][2]/2 > 260:
                self.image_state = 4
            else:
                self.image_state = 5
        else:
            self.image_state = 6
        cv2.waitKey(1)

    def process_image(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]
        mask = np.zeros(h.shape, dtype=np.uint8)
        mask_g = np.zeros(h.shape, dtype=np.uint8)
        mask[((h < 20) | (h > 200)) & (s > 128)] = 255
        mask_g[((h < 150) & (h > 70)) & (s > 128)] = 255
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        _, contours_g, _ = cv2.findContours(mask_g, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        rects_g = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))
        for contour in contours_g:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects_g.append(np.array(rect))
        return rects_g

    def FirstPoint(self):
        self.twist.linear.x = 0.2; self.twist.linear.y = 0; self.twist.linear.z = 0
        self.twist.angular.x = 0; self.twist.angular.y = 0; self.twist.angular.z = 0
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
        r = rospy.Rate(10) # change speed 1fps
        flag = 0
        twist = Twist()
        while not rospy.is_shutdown():
            if self.id_list[-1]==0:
                twist = self.FirstPoint()
                #print(twist_first)
                #self.vel_pub.publish(twist_first)
            elif flag==0:
                self.setGoal(0.2,0.5,-np.pi/4)
                flag = 1
            elif self.near_wall==1:
                twist.linear.x = -self.speed
                twist.angular.z = -0.2
            elif self.near_wall==2:
                twist.linear.x = self.speed
                twist.angular.z = 0.2
            elif self.image_state==3:
                twist.linear.x = 0.1
                twist.angular.z = 0.15
            elif self.image_state==4:
                twist.linear.x = 0.1
                twist.angular.z = -0.15
            elif self.image_state==5:
                twist.linear.x = 0.1
                twist.angular.z = 0
            elif self.image_state==6:
                #twist.linear.x = -0.1
                #twist.angular.z = 0
                self.setGoal(0,-0.5,np.pi/4)
            self.vel_pub.publish(twist)
            #print(twist)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random')
    bot.strategy()


#!/usr/bin/env python
# -*-coding: utf-8 -*-
# BEGIN ALL
import rospy
import time
import math
from deu_maze.msg import LidarMeasure
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

"""
터틀봇의 Odom 데이터를 이용하여 

터틀봇의 방향을 위, 아래, 왼쪽, 오른쪽으로 설정한다.
"""

class MoveRobot:
    def __init__(self):
        self.robotStateDrive = False
        self.robotStateRotate = False
        self.finishAction = True

        sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
        self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.r = rospy.Rate(10)

        #회전을 위한 선언 및 초기화
        self.roll = self.pitch = self.yaw = 0.0
        self.target = 0
        self.command =Twist()

        self.ahead  = 0.0

        self.startTime = time.time()
        self.speed = 0.5
        self.distance = 0.0
        self.drivingForward = True
        self.changeTime = rospy.Time.now()

    def get_rotation(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def setRight(self):
        self.target = 0 # right, self.yaw=0.0
        self.robotStateRotate = True
        self.finishAction = True

    def setLeft(self):
        self.target = 180 # left, self.yaw=3.14
        self.robotStateRotate = True
        self.finishAction = True

    def setUpward(self):
        self.target = 90 # up, yqw=1.57
        self.robotStateRotate = True
        self.finishAction = True

    def setDownward(self):
        self.target = -90 # bottom, self.yaw=-1.57
        self.robotStateRotate = True
        self.finishAction = True

    def setGo(self):
        self.changeTime = rospy.Time.now() + rospy.Duration(1)
        self.robotStateDrive = True
        self.finishAction = True

    def rotate(self):
        if self.target == 180 and self.yaw <= 3.143 and self.yaw >=3.14: # left
            self.command.angular.z = 0
            self.robotStateRotate = False
            self.finishAction = False
        if self.target == 180 and self.yaw >= 3.143: # left
            self.command.angular.z = -7.35
        if self.target == 180 and self.yaw <=3.14: # left
            self.command.angular.z = 7.35
        if self.target == 180 and self.yaw < -1: # left
            self.command.angular.z = -7.35

        if self.target == 90 and self.yaw <= 1.572 and self.yaw >=1.569: # up
            self.command.angular.z = 0
            self.robotStateRotate = False
            self.finishAction = False
        elif self.target == 90 and self.yaw >= 1.572: # up
            self.command.angular.z = -7.35
        elif self.target == 90 and self.yaw <=1.569: # up
            self.command.angular.z = 7.35

        if self.target == -90 and self.yaw >=-1.572 and self.yaw <= -1.569: # bottom
            self.command.angular.z = 0
            self.robotStateRotate = False
            self.finishAction = False
        elif self.target == -90 and self.yaw <=-1.572: # bottom
            self.command.angular.z = 7.35
        elif self.target == -90 and self.yaw > 1.579: # bottom
            self.command.angular.z = 7.35
        elif self.target == -90 and self.yaw >= -1.569: # bottom
            self.command.angular.z = -7.35

        if self.target == 0 and self.yaw <= 0.001 and self.yaw >= -0.001: # right
            self.command.angular.z = 0
            self.robotStateRotate = False
            self.finishAction = False
        elif self.target == 0 and self.yaw >= 0.001:# right
            self.command.angular.z = -7.35
        elif self.target == 0 and self.yaw <= -0.001: # right
            self.command.angular.z = 7.35

    def goAhead(self):
        if self.drivingForward:
            self.command.linear.x = self.speed
            self.distance = abs(self.speed * ( time.time() - self.startTime ))
        else:
            self.command.linear.x = 0
            self.drivingForward = not self.drivingForward
            self.robotStateDrive = False
            self.finishAction = False

        if 1.71 <= self.distance:
            self.drivingForward = not self.drivingForward
            self.distance = 0.0
            self.startTime = time.time()

    def stopRobot(self):
        self.speed = 0

    def execute(self):
        while(self.finishAction == True):
            if(self.robotStateDrive):
                self.goAhead()
            elif(self.robotStateRotate):
                self.rotate()
                self.distance = 0.0
                self.startTime = time.time()
            self.pub.publish(self.command)
            self.r.sleep()

#!/usr/bin/env python
# -*-coding: utf-8 -*-
# BEGIN ALL
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int32


class MoveRobot:
    def __init__(self):
        global send_msg
        send_msg = 1
        self.robot_state_drive = False
        self.robot_state_rotate = False
        self.finish_action = True
        sub = rospy.Subscriber('/odom', Odometry, self.get_rotation)
        self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.r = rospy.Rate(30)

        self.roll = self.pitch = self.yaw = 0.0
        self.target = 0
        self.command = Twist()

        self.driving_forward = True
        self.light_change_time = rospy.Time.now()
        self.kp = 3
        self.speed = 1.2
        self.distance = 0.3

    def get_rotation(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def setRight(self):
        self.target = 180  # right, self.yaw=0.0
        self.robot_state_rotate = True
        self.finish_action = True

    def setLeft(self):
        self.target = 0  # left, self.yaw=3.14
        self.robot_state_rotate = True
        self.finish_action = True

    def setUpward(self):
        self.target = -90  # up, yqw=1.57
        self.robot_state_rotate = True
        self.finish_action = True

    def setDownward(self):
        self.target = 90  # bottom, self.yaw=-1.57
        self.robot_state_rotate = True
        self.finish_action = True

    def setGo(self):
        self.light_change_time = rospy.Time.now() + rospy.Duration(0.34)
        # self.light_change_time = rospy.Time.now() + rospy.Duration(0.34)
        self.robot_state_drive = True
        self.finish_action = True

    def rotate(self):
        target_rad = self.target * math.pi / 180
        if self.target == 180 and self.yaw <= 3.143 and self.yaw >= 3.14:  # left
            self.command.angular.z = self.kp * (target_rad - self.yaw)
            self.robot_state_rotate = False
            self.finish_action = False
        if self.target == 180 and self.yaw >= 3.143:  # left
            self.command.angular.z = self.kp * (target_rad - self.yaw)
        if self.target == 180 and self.yaw <= 3.14:  # left
            self.command.angular.z = self.kp * (target_rad - self.yaw)
        if self.target == 180 and self.yaw < -1:  # left
            self.command.angular.z = self.kp * (target_rad - self.yaw)

        if self.target == 90 and self.yaw <= 1.572 and self.yaw >= 1.569:  # up
            self.command.angular.z = self.kp * (target_rad - self.yaw)
            self.robot_state_rotate = False
            self.finish_action = False
        elif self.target == 90 and self.yaw >= 1.572:  # up
            self.command.angular.z = self.kp * (target_rad - self.yaw)
        elif self.target == 90 and self.yaw <= 1.569:  # up
            self.command.angular.z = self.kp * (target_rad - self.yaw)

        if self.target == -90 and self.yaw >= -1.572 and self.yaw <= -1.569:  # bottom
            self.command.angular.z = self.kp * (target_rad - self.yaw)
            self.robot_state_rotate = False
            self.finish_action = False
        elif self.target == -90 and self.yaw <= -1.572:  # bottom
            self.command.angular.z = self.kp * (target_rad - self.yaw)
        elif self.target == -90 and self.yaw > 1.579:  # bottom
            self.command.angular.z = self.kp * (target_rad - self.yaw)
        elif self.target == -90 and self.yaw >= -1.569:  # bottom
            self.command.angular.z = self.kp * (target_rad - self.yaw)

        if self.target == 0 and self.yaw <= 0.001 and self.yaw >= -0.001:  # right
            self.command.angular.z = self.kp * (target_rad - self.yaw)
            self.robot_state_rotate = False
            self.finish_action = False
        elif self.target == 0 and self.yaw >= 0.001:  # right
            self.command.angular.z = self.kp * (target_rad - self.yaw)
        elif self.target == 0 and self.yaw <= -0.001:  # right
            self.command.angular.z = self.kp * (target_rad - self.yaw)

    def go_ahead(self):
        if self.driving_forward:
            self.command.linear.x = -1 * self.speed
            t0 = rospy.Time.now().to_sec()
            current_distance = 0

            while current_distance < self.distance:

                # Publish the velocity
                self.pub.publish(self.command)
                # Takes actual time to velocity calculus
                t1 = rospy.Time.now().to_sec()
                # Calculates distancePoseStamped
                current_distance = self.speed * (t1 - t0)
            # After the loop, stops the robot
            self.command.linear.x = 0
            self.driving_forward = not self.driving_forward

        else:
            self.command.linear.x = 0
            self.driving_forward = not self.driving_forward
            self.robot_state_drive = False
            self.finish_action = False

        # if rospy.Time.now() > self.light_change_time:
        #     self.driving_forward = not self.driving_forward
    def execute(self):
        # while not rospy.is_shutdown():
        while (self.finish_action == True):
            if (self.robot_state_drive):
                self.go_ahead()
            elif (self.robot_state_rotate):
                self.rotate()
            self.pub.publish(self.command)
            self.r.sleep()


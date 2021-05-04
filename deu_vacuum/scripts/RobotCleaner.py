#!/usr/bin/env python
# -*-coding: utf-8 -*-

import rospy
from deu_vacuum.msg import LidarMeasure
from geometry_msgs.msg import Twist
from MoveRobot import MoveRobot
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import math
import rospy
import actionlib


def get_odom(msg):
    global end_point, turtlebot_pose_y, turtlebot_pose_x, check
    turtlebot_pose_y = msg.pose.pose.position.y
    turtlebot_pose_x = msg.pose.pose.position.x

    if abs(turtlebot_pose_x - (-2.9)) < 0.2 and abs(turtlebot_post_y - (-2.7)) < 0.2:
        end_state = True
    

def move():
    global save_tree
    global range_rear
    global range_ahead
    global range_right
    global range_left
    global robot_state_direction  # 로봇의 현재 방향을 저장하는 변수
    global turtlebot_pose_y
    global turtlebot_pose_x

    ls = [round(turtlebot_pose_x, 1), round(turtlebot_pose_y, 1)]  # Up, Down, left, right
    move_ls = [True, True, True, True]
    print("rear: %s" % range_rear)
    print("left: %s" % range_left)
    print("ahead: %s" % range_ahead)
    print("right: %s" % range_right)
    print(ls)

    check_next_distance = 0.3
    check_wall_distance = 0.53
    check_node_accuracy = 0.25
    state_down = 0
    state_up = 1
    state_left = 2
    state_right = 3
    end_state = 0
    end_pose = [-2.9,-2.7]

    save_tree.append(ls)

    for i in save_tree:
        if abs(i[0]) < 0.25 and abs(i[1] - (-2.7))<0.25:
            end_state = 2
            break
        if check_node_accuracy > abs(
                round(turtlebot_pose_x - round(i[0], 1) - check_next_distance, 1)) and check_node_accuracy > abs(
                round(turtlebot_pose_y, 1) - round(i[1], 1)):
            move_ls[2] = False
        if check_node_accuracy > abs(
                round(turtlebot_pose_x + check_next_distance, 1) - round(i[0], 1)) and check_node_accuracy > abs(
                round(turtlebot_pose_y, 1) - round(i[1], 1)):
            move_ls[3] = False
        if check_node_accuracy > abs(
                round(turtlebot_pose_y + check_next_distance, 1) - round(i[1], 1)) and check_node_accuracy > abs(
                round(turtlebot_pose_x, 1) - round(i[0], 1)):
            move_ls[1] = False
        if check_node_accuracy > abs(
                round(turtlebot_pose_y - check_next_distance, 1) - round(i[1], 1)) and check_node_accuracy > abs(
                round(turtlebot_pose_x, 1) - round(i[0], 1)):
            move_ls[0] = False

     if end_state:
         print("Call End")
         save_tree.reverse()
         while ls[0] is not 0.0 and ls[1] is not 0.0:
             for i in save_tree:
        
                 if ls[0] < i[0] and ls[1] == i[1]: # right
                     mr.setRight()
        
                 elif ls[0] > i[0] and ls[1] == i[1]: #left
                     mr.setLeft()
        
                 elif ls[1] > i[1] and ls[0] == i[0]: # up
                     mr.setUpward()
        
                 elif ls[1] < i[1] and ls[0] == i[0]: # down
                     mr.setDownward()
                 mr.execute()
                 mr.setGo()
                 mr.execute()
                 ls[0] = i[0]
                 ls[1] = i[1]
                 rospy.sleep(0.5)
        # end_state = 2





    if robot_state_direction is state_up:  # Up
        if range_rear < check_wall_distance:
            move_ls[state_down] = False
        if range_right < check_wall_distance:
            move_ls[state_right] = False
        if range_left < check_wall_distance:
            move_ls[state_left] = False
        if range_ahead < check_wall_distance:
            move_ls[state_up] = False

    elif robot_state_direction is state_down:  # Down
        if range_rear < check_wall_distance:
            move_ls[state_up] = False
        if range_right < check_wall_distance:
            move_ls[state_left] = False
        if range_left < check_wall_distance:
            move_ls[state_right] = False
        if range_ahead < check_wall_distance:
            move_ls[state_down] = False

    elif robot_state_direction is state_left:  # Left
        if range_rear < check_wall_distance:
            move_ls[state_right] = False
        if range_right < check_wall_distance:
            move_ls[state_up] = False
        if range_left < check_wall_distance:
            move_ls[state_down] = False
        if range_ahead < check_wall_distance:
            move_ls[state_left] = False

    elif robot_state_direction is state_right:  # Right
        if range_rear < check_wall_distance:
            move_ls[state_left] = False
        if range_right < check_wall_distance:
            move_ls[state_down] = False
        if range_left < check_wall_distance:
            move_ls[state_up] = False
        if range_ahead < check_wall_distance:
            move_ls[state_right] = False

    print(move_ls)

    if move_ls[state_up]:
        if robot_state_direction is not state_up:
            robot_state_direction = state_up
            mr.setUpward()
            mr.execute()
        mr.setGo()
        mr.execute()

    elif move_ls[state_down]:
        if robot_state_direction is not state_down:
            robot_state_direction = state_down
            mr.setDownward()
            mr.execute()
        mr.setGo()
        mr.execute()

    elif move_ls[state_left]:
        if robot_state_direction is not state_left:
            robot_state_direction = state_left
            mr.setLeft()
            mr.execute()
        mr.setGo()
        mr.execute()

    elif move_ls[state_right]:
        if robot_state_direction is not state_right:
            robot_state_direction = state_right
            mr.setRight()
            mr.execute()
        mr.setGo()
        mr.execute()
    else:
        if robot_state_direction == state_down:  # 아래
            if range_ahead < check_wall_distance:

                if range_left > check_wall_distance:
                    mr.setRight()
                    robot_state_direction = state_right

                elif range_right > check_wall_distance:
                    mr.setLeft()
                    robot_state_direction = state_left
                else:
                    mr.setUpward()
                    robot_state_direction = state_up
            else:
                mr.setGo()
            mr.execute()

        elif robot_state_direction == state_left:  # 왼쪽
            if range_ahead < check_wall_distance:

                if range_left > check_wall_distance:
                    mr.setDownward()
                    robot_state_direction = state_down

                elif range_right > check_wall_distance:
                    mr.setUpward()
                    robot_state_direction = state_up

                else:
                    mr.setRight()
                    robot_state_direction = state_right
            else:
                mr.setGo()
            mr.execute()

        elif robot_state_direction == state_up:  # 위
            if range_ahead < check_wall_distance:
                if range_left > check_wall_distance:
                    mr.setLeft()
                    robot_state_direction = state_left
                elif range_right > check_wall_distance:
                    mr.setRight()
                    robot_state_direction = state_right
                else:
                    mr.setDownward()
                    robot_state_direction = state_down
            else:
                mr.setGo()
            mr.execute()

        elif robot_state_direction == state_right:  # 오른쪽
            if range_ahead < check_wall_distance:
                if range_right > check_wall_distance:
                    mr.setDownward()
                    robot_state_direction = state_down

                elif range_left > check_wall_distance:
                    mr.setUpward()
                    robot_state_direction = state_up
                else:
                    mr.setLeft()
                    robot_state_direction = state_left
            else:
                mr.setGo()
            mr.execute()

def cal_meter():
    return len(save_tree) * 0.3

def cal_overlapping_meter():
    count = 0
    for i in range(len(save_tree)):
        for j in range(len(save_tree)):
            if save_tree[i][0] == save_tree[j][0] and save_tree[i][1] == save_tree[j][1]:
                count += 1
    return count*0.3

def cal_time():
    return len(save_tree) * 0.75

def lidar_data_callback(floatmsg):
    global test
    global init_state
    global end_point
    global robot_state_direction
    global save_direction
    global turtlebot_pose_y
    global turtlebot_pose_x
    global check
    global send_msg
    global range_rear
    global range_ahead
    global range_right
    global range_left
    global end_state
    range_rear = floatmsg.range_rear
    range_ahead = floatmsg.range_ahead
    range_right = floatmsg.range_right
    range_left = floatmsg.range_left


rospy.init_node('escape_algorithm')

mr = MoveRobot()
tree_ls = []
mr.setDownward()
mr.execute()
robot_state_direction = 1
save_direction = []
save_tree = []
init_state = True
end_point = False
check = 0
turtlebot_pose_y = 0
turtlebot_pose_x = 0
range_rear = 0
range_ahead = 0
range_right = 0
range_left = 0
end_state = False
scan_sub = rospy.Subscriber('lidar_measure', LidarMeasure, lidar_data_callback, queue_size=1)
odom_sub = rospy.Subscriber('/odom', Odometry, get_odom, queue_size=1)

while not end_state == 2:
    move()
    print "청소 중복 면적 : %f M(2)" % cal_overlapping_meter()
    print "청소 면적 : %f M(2)" % cal_meter()
    print "청소 시간 : %f (s)" % cal_time()
    rospy.sleep(0.5)

rospy.spin()

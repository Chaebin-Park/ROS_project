#!/usr/bin/env python
# -*-coding: utf-8 -*-

import rospy
import sys
from collections import deque
from deu_maze.msg import LidarMeasure
from geometry_msgs.msg import Twist
from MoveRobot import MoveRobot
from nav_msgs.msg import Odometry


"""
터틀봇이 구독받은 scan값에 따라 "전방, 좌, 우, 후방"을 우선순위로 이동하게됩니다.
터틀봇은 이동할 때마다 방향을 stack에 기록합니다.
미로 탈출 후 기록한 방향을 pop 하며 출발점으로 되돌아갑니다.

"""


def get_odom(msg):  # 터틀봇이 출구로 빠져나간것을 지속적으로 확인
    global endState, turtlebot_pose_y, turtlebot_pose_x, check
    turtlebot_pose_y = msg.pose.pose.position.y
    turtlebot_pose_x = msg.pose.pose.position.x
    if turtlebot_pose_y < -9.5 and len(fork) is 0:
        check = 1
        endState = True


def lidar_data_callback(floatmsg):
    global initState            # 터틀봇을 초기 입구로 보내는 변수
    global endState             # 출구에 도착했는지 확인하는 변수
    global robot_state_direction  # 로봇의 현재 방향을 저장하는 변수
    global turtlebot_pose_y      # 터틀봇의 pose.y의 위치를 확인
    global turtlebot_pose_x
    global check                 # 터틀봇 pose.y에 따른 이벤트 발생 변수
    global fork
    global stack
    global isBlocked
    global aDown
    global aLeft
    global aUp
    global aRight
    global wallDistance
    global tmpCoor
    global tmpDir

    if check is 1 and turtlebot_pose_y <= -9.5:  # 탈출 후 미로 재진입
        print("======= maze again!!!! =======")
        stack.pop()
        mr.setUpward()
        mr.execute()
        mr.setGo()
        mr.execute()
        robot_state_direction = aUp
    elif endState and turtlebot_pose_y > -9.5:
        if stack:
            robot_state_direction = (stack.pop() + 2) % 4
            if robot_state_direction is aUp:
                mr.setUpward()
            elif robot_state_direction is aDown:
                mr.setDownward()
            elif robot_state_direction is aLeft:
                mr.setLeft()
            elif robot_state_direction is aRight:
                mr.setRight()
            mr.execute()
            mr.setGo()
            mr.execute()
            print(stack)
        else:
            print("complete!")
            mr.stopRobot()
            sys.exit()

    elif initState:  # 초기 시작 미로 안으로 진입
        print("======= start!!!! =======")
        mr.setDownward()    # 아래방향
        mr.execute()        # 아래방향 진행
        robot_state_direction = aDown
        stack.append(aDown)
        mr.setGo()          # 전진
        mr.execute()        # 전진 진행
        initState = False
    else:            # 초기 상태 후 미로를 탈출하는 알고리즘
        print("[fork]    {}".format(fork))
        print("[stack]   {}".format(stack))
        print("")

        if robot_state_direction is aDown:     # 아래방향 바라보고 있을 때
            if floatmsg.range_ahead < wallDistance:     # 전방 벽 감지
                if floatmsg.range_right > wallDistance and floatmsg.range_left > wallDistance:  # 양쪽 길있음
                    fork.append([turtlebot_pose_x, turtlebot_pose_y])
                    stack.append(aLeft)
                    stack.append(aRight)
                elif floatmsg.range_right > wallDistance:  # 오른쪽 길 있음
                    stack.append(aLeft)
                elif floatmsg.range_left > wallDistance:  # 왼쪽 길 있음
                    stack.append(aRight)
                if floatmsg.range_right < wallDistance and floatmsg.range_left < wallDistance:   # 막다른 길
                    mr.setUpward()
                    mr.execute()
                    isBlocked = True
            elif floatmsg.range_ahead >= wallDistance:  # 전방 벽 없음
                stack.append(aDown)
                if floatmsg.range_left >= wallDistance:  # 전방, 왼쪽 갈림길
                    stack.append(aRight)
                    fork.append([turtlebot_pose_x, turtlebot_pose_y])
                if floatmsg.range_right >= wallDistance:  # 전방, 오른쪽 갈림길
                    stack.append(aLeft)
                    fork.append([turtlebot_pose_x, turtlebot_pose_y])

        elif robot_state_direction is aLeft:   # 왼쪽방향 바라보고 있을 때
            if floatmsg.range_ahead < wallDistance:     # 전방 벽 감지
                if floatmsg.range_right > wallDistance and floatmsg.range_left > wallDistance:  # 양쪽 길있음
                    fork.append([turtlebot_pose_x, turtlebot_pose_y])
                    stack.append(aUp)
                    stack.append(aDown)
                elif floatmsg.range_right > wallDistance:  # 오른쪽 길 있음
                    stack.append(aUp)
                elif floatmsg.range_left > wallDistance:  # 왼쪽 길 있음
                    stack.append(aDown)
                if floatmsg.range_right < wallDistance and floatmsg.range_left < wallDistance:   # 막다른 길
                    mr.setRight()
                    mr.execute()
                    isBlocked = True
            elif floatmsg.range_ahead >= wallDistance:  # 전방 벽 없음
                stack.append(aLeft)
                if floatmsg.range_left >= wallDistance:  # 전방, 왼쪽 갈림길
                    fork.append([turtlebot_pose_x, turtlebot_pose_y])
                    stack.append(aDown)
                if floatmsg.range_right >= wallDistance:  # 전방, 오른쪽 갈림길
                    fork.append([turtlebot_pose_x, turtlebot_pose_y])
                    stack.append(aUp)

        elif robot_state_direction is aRight:    # 오른쪽방향 바라보고 있을 때
            if floatmsg.range_ahead < wallDistance:     # 전방 벽 감지
                if floatmsg.range_right > wallDistance and floatmsg.range_left > wallDistance:  # 양쪽 길있음
                    fork.append([turtlebot_pose_x, turtlebot_pose_y])
                    stack.append(aDown)
                    stack.append(aUp)
                elif floatmsg.range_right > wallDistance:  # 오른쪽 길 있음
                    stack.append(aDown)
                elif floatmsg.range_left > wallDistance:  # 왼쪽 길 있음
                    stack.append(aUp)
                if floatmsg.range_right < wallDistance and floatmsg.range_left < wallDistance:   # 막다른 길
                    mr.setLeft()
                    mr.execute()
                    isBlocked = True
            elif floatmsg.range_ahead >= wallDistance:  # 전방 벽 없음
                stack.append(aRight)
                if floatmsg.range_left >= wallDistance:  # 전방, 왼쪽 갈림길
                    fork.append([turtlebot_pose_x, turtlebot_pose_y])
                    stack.append(aUp)
                if floatmsg.range_right >= wallDistance:  # 전방, 오른쪽 갈림길
                    fork.append([turtlebot_pose_x, turtlebot_pose_y])
                    stack.append(aDown)

        elif robot_state_direction is aUp:    # 위쪽방향 바라보고 있을 때
            if floatmsg.range_ahead < wallDistance:     # 전방 벽 감지
                if floatmsg.range_right > wallDistance and floatmsg.range_left > wallDistance:  # 양쪽 길있음
                    fork.append([turtlebot_pose_x, turtlebot_pose_y])
                    stack.append(aRight)
                    stack.append(aLeft)
                if floatmsg.range_right > wallDistance:  # 오른쪽 길 있음
                    stack.append(aRight)
                if floatmsg.range_left > wallDistance:  # 왼쪽 길 있음
                    stack.append(aLeft)
                if floatmsg.range_right < wallDistance and floatmsg.range_left < wallDistance:   # 막다른 길
                    mr.setUpward()
                    mr.execute()
                    isBlocked = True
            elif floatmsg.range_ahead >= wallDistance:  # 전방 벽 없음
                stack.append(aUp)
                if floatmsg.range_left >= wallDistance:  # 전방, 왼쪽 갈림길
                    fork.append([turtlebot_pose_x, turtlebot_pose_y])
                    stack.append(aLeft)
                if floatmsg.range_right >= wallDistance:  # 전방, 오른쪽 갈림길
                    fork.append([turtlebot_pose_x, turtlebot_pose_y])
                    stack.append(aRight)

        if isBlocked:   # 막다른 길 도착
            while True:  # 가장 최근의 분기점으로 복귀
                robot_state_direction = (stack.pop() + 2) % 4
                if robot_state_direction is aUp:
                    mr.setUpward()
                elif robot_state_direction is aDown:
                    mr.setDownward()
                elif robot_state_direction is aRight:
                    mr.setRight()
                elif robot_state_direction is aLeft:
                    mr.setLeft()
                mr.execute()
                mr.setGo()
                mr.execute()
                if abs(turtlebot_pose_x - fork[0][0]) < 0.5 and abs(turtlebot_pose_y - fork[0][1]) < 0.5 or abs(tmpCoor[0] - turtlebot_pose_x) < 0.5 and abs(tmpCoor[1] - turtlebot_pose_y) < 0.5:
                    fork.popleft()
                    break
                print("[blocked] {}".format(stack))
            if abs(tmpCoor[0] - turtlebot_pose_x) < 0.5 and abs(tmpCoor[1] - turtlebot_pose_y) < 0.5:
                robot_state_direction = tmpDir
            # 다음 경로 방향으로 방향 전환
            else :
                robot_state_direction = stack.pop()

            if robot_state_direction is aUp:
                mr.setUpward()
            elif robot_state_direction is aDown:
                mr.setDownward()
            elif robot_state_direction is aRight:
                mr.setRight()
            elif robot_state_direction is aLeft:
                mr.setLeft()

            mr.execute()
            stack.append(robot_state_direction)
            isBlocked = False

        elif len(fork) >= 2:
            stack.pop()
            stack.pop()
            robot_state_direction = (robot_state_direction + 2) % 4
            if robot_state_direction is aUp:
                mr.setUpward()
            elif robot_state_direction is aDown:
                mr.setDownward()
            elif robot_state_direction is aRight:
                mr.setRight()
            elif robot_state_direction is aLeft:
                mr.setLeft()
            mr.execute()
            mr.setGo()
            mr.execute()

            if abs(turtlebot_pose_x - fork[0][0]) < 0.5 and abs(turtlebot_pose_y - fork[0][1]) < 0.5:
                tmpCoor = fork.popleft()
                tmpDir = stack.pop()
                print("")
                print("[find fork] tmpCoor : {} tmpDir : {}".format(tmpCoor, tmpDir))
                print("")
            else:
                stack.pop()
                while True:  # 가장 최근의 분기점으로 복귀
                    print("[fork_back] {}".format(stack))
                    tmpDir = stack.pop()
                    robot_state_direction = (tmpDir + 2) % 4
                    if robot_state_direction is aUp:
                        mr.setUpward()
                    elif robot_state_direction is aDown:
                        mr.setDownward()
                    elif robot_state_direction is aRight:
                        mr.setRight()
                    elif robot_state_direction is aLeft:
                        mr.setLeft()
                    mr.execute()
                    mr.setGo()
                    mr.execute()
                    if abs(turtlebot_pose_x - fork[0][0]) < 0.5 and abs(turtlebot_pose_y - fork[0][1]) < 0.5:
                        tmpCoor = fork.popleft()
                        print("")
                        print("[find fork] tmpCoor : {} tmpDir : {}".format(tmpCoor, tmpDir))
                        print("")
                        break
        else:   # 막다른 길이 아니고 저장된 분기점이 없을 때 정상 진행
            robot_state_direction = stack[len(stack)-1]
            if robot_state_direction is aUp:
                mr.setUpward()
            elif robot_state_direction is aDown:
                mr.setDownward()
            elif robot_state_direction is aRight:
                mr.setRight()
            elif robot_state_direction is aLeft:
                mr.setLeft()
            mr.execute()
        mr.setGo()
        mr.execute()

rospy.init_node('escape_algorithm')

mr = MoveRobot()
aDown = 0
aLeft = 1
aUp = 2
aRight = 3
wallDistance = 1.5
tmpCoor = [0,0]
tmpDir = 0

stack = list()
fork = deque()
visited = list()
robot_state_direction = 1  # 0 아래, 1 왼쪽, 2 위, 3 오른쪽  -> 터틀봇 기준

isBlocked = False
isVisited = False
isJustGo = True

initState = True
endState = False
check = 0
turtlebot_pose_y = 0
turtlebot_pose_x = 0

scan_sub = rospy.Subscriber('lidar_measure', LidarMeasure, lidar_data_callback, queue_size=1)
odom_sub = rospy.Subscriber('/odom', Odometry, get_odom, queue_size=1)

# Create a publisher that moves the robot
pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

# Create a global variable for publising a Twist ("cmd_vel") message 
move = Twist()

# Keep the program running
rospy.spin()

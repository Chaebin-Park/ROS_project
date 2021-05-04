#!/usr/bin/env python
# -*-coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from deu_maze.msg import LidarMeasure

"""
터틀봇 5m 이내 영역을 lidar로 측정한 scan값을 구독하여 가져와 가공하고,

새로 정의한 LidarMeasure 메시지에 담아서 발행한다.
"""

# BEGIN MEASUREMENT
def scan_callback(msg):
  floatmsg = LidarMeasure()
  floatmsg.range_ahead = msg.ranges[len(msg.ranges)/2]
  floatmsg.range_rear = msg.ranges[len(msg.ranges)*0]
  floatmsg.range_right = msg.ranges[len(msg.ranges)/4]
  floatmsg.range_left = msg.ranges[len(msg.ranges)/4*3]

  scan_pub.publish(floatmsg)
  # END MEASUREMENT

rospy.init_node('lidar_measurement')
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
scan_pub = rospy.Publisher('lidar_measure', LidarMeasure, queue_size=1)
rospy.spin()
# END ALL

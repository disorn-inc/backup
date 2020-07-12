#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
import os 
import cv2
from enum import Enum
from std_msgs.msg import UInt8, Float64, String
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
class ridar_test():
    def __init__(self):
        self.sub_scan_obstacle = rospy.Subscriber('/scan', LaserScan, self.cbScanObstacle, queue_size=1)
        self.obstacle_detect = rospy.Publisher('/obs_de', UInt8, queue_size = 1)
    def cbScanObstacle(self, scan):
        angle_scan = 60
        scan_start = 270 - angle_scan
        scan_end = 270 + angle_scan
        threshold_distance = 0.2
        is_obstacle_detected = 0

        for i in range(scan_start, scan_end):
            if scan.ranges[i] < threshold_distance and scan.ranges[i] > 0.01:
                is_obstacle_detected = 1
        self.is_obstacle_detected = is_obstacle_detected
        print(is_obstacle_detected)
        self.obstacle_detect.publish(is_obstacle_detected)
    def main(self):
        rospy.spin()
if __name__ == '__main__':
    rospy.init_node('ridar_test')
    node = ridar_test()
    node.main()

#!/usr/bin/env python
from __future__ import print_function
import cv2
import numpy as np
import statistics
import sys
import rospy
from std_msgs.msg import String,Float64,Int64
import roslib
roslib.load_manifest('usb_cam')
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from time import time

previous = time()
delta = 0
d=1
cam = cv2.VideoCapture(0)

while True:
   _, img = cam.read()
   filename = "image/%d.jpg"%d
   cv2.imshow("Frame", img)
   current = time()
   delta += current - previous
   previous = current

   if delta > 1:
     cv2.imwrite(filename,img)
     d+=1
     delta = 0
   print(delta)
   cv2.waitKey(1)



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

bridge = CvBridge()
d=0


def intitial():
    rospy.Subscriber("/usb_cam/image_raw",Image,Getcam)
    #rospy.Subscriber("/usb_cam/image_raw",Image,=main)


def Getcam(data):
    
  d=0
  d=d+1
  cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
  filename = "image/file_%d.jpg"%d

previous = time()
delta = 0


while True:
  
    current = time()
    delta += current - previous
    previous = current

   
    if delta > 3:
       
         cv2.imwrite(filename,cv_image) 
         print("down")
         delta = 0
  

cv2.imshow('frame',cv_image)
cv2.waitKey(3)
return cv_image

def main(args):
  rospy.init_node('image_node', anonymous=True)
  intitial()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)
    
#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import Int16
import random
import roslib
import sys
import cv2
import numpy as np
import statistics
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
run = 1
#Corgi State
P1 = [470,120]
P2 = [470,70]
P3 = [500,120]
P4 = [500,70]
#Crop border

lower_red = np.array([0, 0, 200])
upper_red = np.array([5, 255, 255])

lower_green = np.array([40,20,100])
upper_green = np.array([100,255,255])

lower_pink = np.array([255, 255, 255])
upper_pink = np.array([255, 255, 255])
#Magenta Increase 30 "H" range 
def trafficgreen(frame):
  global run
  y1=70
  y2=120
  x1=370
  x2=420
  kernel = np.ones((5,5),np.float32)/25
  crop_frame = frame[ P2[1]:P1[1] , P1[0]:P3[0] ]
  res = cv2.filter2D(frame,-1,kernel)
  hsv = crop_frame 
  mask = cv2.inRange(hsv, lower_red, upper_red)
  mask2 = cv2.inRange(hsv, lower_green, upper_green)
  red_traffic = cv2.bitwise_and(crop_frame,crop_frame,mask=mask)
  green_traffic = cv2.bitwise_and(crop_frame,crop_frame,mask=mask2)
  
  if (np.sum(red_traffic)/255) > 80 :
    run = 3
    return red_traffic
  elif (np.sum(green_traffic)/255) > 50 :
    run = 2
    return green_traffic
  else :
	  run = 1
  return frame

def Detect_Terminal(frame):
    global run
    height = int(frame.shape[0])
    weight = int(frame.shape[1])
    crop_img = frame[int((height*0.4)):int((height*0.6)), 0:weight]
    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)    
    pink_mask = cv2.inRange(hsv, lower_pink, upper_pink)
    res = cv2.bitwise_and(crop_img,crop_img,mask=pink_mask)    
    number1 = np.sum(pink_mask)
    number2 = np.sum(res)
    if number1<number2:
        run = 4
    return pink_mask

class image_converter:
  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/image",Image,self.callback)
    self.image_pub_Purple = rospy.Publisher("/image/traffic/light/Purple",Image,queue_size=10)
    self.image_pub_crop = rospy.Publisher("/image/traffic/light/crop",Image,queue_size=10)
    self.image_pub_light = rospy.Publisher("/image/traffic/light",Image,queue_size=10)
    self.pub = rospy.Publisher('traffic/status', Int16, queue_size=10)
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
        cv_image=cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        cv2.circle(cv_image, (P1[0],P1[1]), 5, (255, 255, 255), -1)
        cv2.circle(cv_image, (P2[0],P2[1]), 5, (255, 255, 255), -1)
        cv2.circle(cv_image, (P3[0],P3[1]), 5, (255,255, 255), -1)
        cv2.circle(cv_image, (P4[0],P4[1]), 5, (255, 255, 255), -1)
        image=trafficgreen(cv_image)
        res = Detect_Terminal(cv_image)
       # cv_image=cv2.cvtColor( cv_image, cv2.COLOR_BGR2GRAY)
  #      image=cv2.cvtColor( image, cv2.COLOR_BGR2GRAY)
        cv_image=cv2.resize(cv_image,(0,0),fx=0.5,fy=0.5)
    print(run)
    status = run
    try:
        self.image_pub_Purple.publish(self.bridge.cv2_to_imgmsg(res,"mono8"))
        self.image_pub_light.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))
        self.image_pub_crop.publish(self.bridge.cv2_to_imgmsg(image,"bgr8"))
        self.pub.publish(status)
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('image_traffice', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

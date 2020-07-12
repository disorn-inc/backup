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
bridge = CvBridge()



lower_red = np.array([0, 50, 180])
upper_red = np.array([5, 255, 255])

lower_green = np.array([80,0,40])
upper_green = np.array([180,255,255])

def intitial():
    rospy.Subscriber("/usb_cam/image_raw",Image,trafficgreen)


def trafficgreen(data):
  frame = bridge.imgmsg_to_cv2(data, "bgr8")
  global run
  y1=70
  y2=120
  x1=370
  x2=420
  kernel = np.ones((5,5),np.float32)/25
  crop_frame = frame[12:57 , 152:253]
  res = cv2.filter2D(frame,-1,kernel)
  hsv = cv2.cvtColor(crop_frame, cv2.COLOR_BGR2HSV) 
  mask = cv2.inRange(hsv, lower_red, upper_red)
  mask2 = cv2.inRange(hsv, lower_green, upper_green)        
  #mask = cv2.bitwise_or(mask_red,mask_green)
  #masked = cv2.bitwise_and(crop_frame ,crop_frame, mask = mask)
  
  red_traffic = cv2.bitwise_and(crop_frame,crop_frame,mask = mask)
  green_traffic = cv2.bitwise_and(crop_frame,crop_frame,mask= mask2)
  #rospy.loginfo(np.sum(red_traffic)/255)
  #print(np.sum(red_traffic)/255)
  if (np.sum(red_traffic)/255) > 500 :
    run = 0
    result = red_traffic
  elif (np.sum(green_traffic)/255) > 100 :
    run = 1
    result = green_traffic
  else :
    run = 2
    result = hsv
  cv2.imshow('normal', frame)
  cv2.imshow('light', result)
  cv2.waitKey(3)
  print(run)




def main(args):
  rospy.init_node('light_node', anonymous=True)
  intitial()
  
  # result,detect_color = Getcam()
  # cv2.line(result, (int(result.shape[1]*0.5), y_bot),(int(result.shape[1]*0.5), y_top), (0, 0, 255), 1,  8)
  # blobs[0], keypoint[0] = bloblane(detect_color, multiply_y[0],10)
  # blobs[1], keypoint[1] = bloblane(detect_color, multiply_y[1],10)
  # blobs[2], keypoint[2] = bloblane(detect_color, multiply_y[2],10)
  # blobby(keypoint,0)
  # blobby(keypoint,1)
  # blobby(keypoint,2)
  # BLOB=np.concatenate((blobs[2],blobs[1],blobs[0]),axis=0)
  # Detect_Curve(result)
  # Servo_morter = Cal_Servo(result,x_C[2])
  # result =np.concatenate((result,BLOB),axis=0)
  # detect_color  = cv2.resize(BLOB, (0,0), fx=0.5, fy=0.5)
  # result = cv2.resize(result, (0,0), fx=0.5, fy=0.5)
  # cv2.circle(frame,(0,int(frame.shape[0]*0.82)),1,(0,0,255),2)
  # cv2.circle(frame,(int(frame.shape[1]),int(frame.shape[0]*0.8)),1,(0,0,255),2)
  # cv2.circle(frame,(int(frame.shape[1]*0.15),int(frame.shape[0]*0.6)),1,(0,0,255),2)
  # cv2.circle(frame,(int(frame.shape[1]*0.87),int(frame.shape[0]*0.6)),1,(0,0,255),2)
  # cv2.imshow('frame',frame[0:int(frame.shape[0]),0:int(frame.shape[1])])
  # cv2.imshow('roi',result)
  # cv2.imshow('color',detect_color)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)
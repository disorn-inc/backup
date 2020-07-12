#!/usr/bin/env python
import cv2
import liboCams
import numpy as np
import statistics
import sys
import rospy
from std_msgs.msg import String,Float64,Int64

#run = 1
#Corgi State
#Crop border
lower_red = np.array([0, 0, 200])
upper_red = np.array([5, 255, 255])

lower_green = np.array([80,0,40])
upper_green = np.array([180,255,255])

#lower_green = np.array([103,0,100])
#upper_green = np.array([125,255,255])


lower_yellow = np.array([25, 80,70])
upper_yellow = np.array([40, 232, 255])

lower_white = np.uint8([0, 200, 0])
upper_white = np.uint8([255, 255, 255])

Servo_morter = 0
slope = 1
intercept = 1
xpointbot = 0
xpointtop = 0 
xpointmid = 0
x_C = [0, 0, 0]
B1 = [0,0]
B2 = [0,0]
B3 = [0,0]
B4 = [0,0]
multiply_y = [0.9,0.8,0.7]
Size_crop = [0,0]
blobs = [0, 0, 0]
blob_c = [0, 0, 0]
keypoint = [0, 0, 0]
kernel = np.ones((3,3), np.uint8)
MODE_COLOR = 0

class OCamInterface(object):
    """Class For interfacing with oCam Camera
    and Convert to sensor_msgs/Image
    """
    def __init__(self):
        # Parameters
        self.verbose_mode = False
        # OCam
        self.camera_path = None
        self.camera = None
        self.camera_format_list = None
        self.camera_control_list = True
        # Initialize
        self.init_camera(self.verbose_mode)

    def init_camera(self, verbose_flag):
        # Find Camera
        self.camera_path = liboCams.FindCamera('oCam')
        if self.camera_path is None:
            print "Cannot find DevPath : please connect Camera"
            return False

        # Instantiate Camera with Verbose for collecting data
        self.camera = liboCams.oCams(
       	    self.camera_path,
            verbose=1
        )
        # Collecting Data
        self.gathering_camera_data(self.camera)

        ctrlist = self.camera.GetControlList()

        # Camera setting reassign (?)
        for i in range(len(self.camera_format_list)):
	    self.camera.Set(self.camera_format_list[1])

        self.camera_name = self.camera.GetName()

        # Mute Data Printing if verbose is false
        if verbose_flag is False:
            self.camera.Close()
            self.camera = liboCams.oCams(
                self.camera_path,
                verbose=0
            )

        # Start Camera instance !
        self.camera.Start()
        #code for camera control

        return self.camera

    def gathering_camera_data(self, camera):
        """Collecting Camera Data from driver
Arguments:
            camera {libOCams Camera} -- Camera instance from driver
        """
        self.camera_format_list = camera.GetFormatList()
        self.camera_control_list = camera.GetControlList()

    def get_image(self):
        """Get the image Data and publish into ROS World :)
        """
        # Get Image from Camera
        camera_mat = self.camera.GetFrame()
        image = cv2.cvtColor(camera_mat, cv2.COLOR_BayerGB2BGR)
        rgb = cv2.resize(image,(0,0),fx=0.2,fy=0.2)
        return rgb

def trafficgreen(frame):
  global run
  y1=70
  y2=120
  x1=370
  x2=420
  kernel = np.ones((5,5),np.float32)/25
  crop_frame = frame [12:57 , 152:253]
  res = cv2.filter2D(frame,-1,kernel)
  hsv = cv2.cvtColor(crop_frame, cv2.COLOR_BGR2HSV) 
  mask = cv2.inRange(hsv, lower_red, upper_red)
  mask2 = cv2.inRange(hsv, lower_green, upper_green)
  #mask = cv2.bitwise_or(mask_red,mask_green)
  #masked = cv2.bitwise_and(crop_frame ,crop_frame, mask = mask)
  
  red_traffic = cv2.bitwise_and(crop_frame,crop_frame,mask = mask)
  green_traffic = cv2.bitwise_and(crop_frame,crop_frame,mask= mask2)
  rospy.loginfo(np.sum(red_traffic)/255)
  if (np.sum(red_traffic)/255) > 150 :
    run = 0
    return red_traffic
  elif (np.sum(green_traffic)/255) > 100 :
    run = 1
    return green_traffic
  else :
	  run = 2

  return hsv


def Getcam(cv_image):
    global x_mid
    global y_bot
    global y_top
    global y_mid
    global main_Y
    global MODE_COLOR
    point(cv_image)
    pts1 = np.float32([[B1], [B2], [B3], [B4]])
    #pts1 = np.float32([[67,87],[157,92],[0,136],[163,134]])
    pts2 = np.float32([[0, 0], [Size_crop[0], 0], [0, Size_crop[1]], [Size_crop[0], Size_crop[1]]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    result = cv2.warpPerspective(cv_image, matrix, (Size_crop[0], Size_crop[1]))
    result = Crop_to_Cal(result)
    result = cv2.resize(result, (0,0), fx=2, fy=2) 
    if MODE_COLOR == 0 :
      hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    else :
      hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, lower_white, upper_white)
    
    crop = cv2.bitwise_and(result, result, mask=mask)
    crop = cv2.morphologyEx(crop, cv2.MORPH_OPEN, kernel)
    crop = cv2.dilate(crop,kernel,iterations = 1)
    crop = cv2.morphologyEx(crop, cv2.MORPH_CLOSE, kernel)

    x_mid = int(result.shape[1]*0.5)
    y_bot = int(result.shape[0]*(multiply_y[0]+0.025))
    y_top = int(result.shape[0]*(multiply_y[2]+0.025))
    y_mid = int((y_bot+y_top)/2)
    main_Y = [result.shape[0]*(multiply_y[0]+0.025),
                result.shape[0]*(multiply_y[1]+0.025),
                result.shape[0]*(multiply_y[2]+0.025)]
    return result,crop

def point(cv_image):
    global B1
    global B2
    global B3
    global B4
    global Size_crop
    if MODE_COLOR == 0:
        B1 = [67,87]
        B2 = [157,92]
        B3 = [0,136]
        B4 = [163,134]
    else:
        B1 = []
        B2 = []
        B3 = []
        B4 = []

    
    #B1[0] = int(cv_image.shape[1]*0.3125)
    #B2[0] = int(cv_image.shape[1]*0.6875)
    #B3[0] = int(cv_image.shape[1]*0)
    #B4[0] = int(cv_image.shape[1]*1)
    #B1[1] = int(cv_image.shape[0]*0.45)
    #B2[1] = int(cv_image.shape[0]*0.45)
    #B3[1] = int(cv_image.shape[0]*0.694)
    #B4[1] = int(cv_image.shape[0]*0.694) 

    Size_crop[0] = int(cv_image.shape[1]*0.4)
    Size_crop[1] = int(cv_image.shape[0])

def Crop_to_Cal(result):
    A1 = [0 , int(result.shape[0])]
    A2 = [int(result.shape[1]*0.4) , int(result.shape[0])]
    A3 = [0, int(result.shape[0]*0.6)]
    A4 = [int(result.shape[1]*0.4) , int(result.shape[0]*0.6)]
    crop_img = result[A3[1]:A1[1], A1[0]:A2[0]]
    return crop_img

def bloblane(image, y_point,minblobs):
    if minblobs == 0:
        minblobs  = image.shape[1]*0.08
    xp =  int(image.shape[1])
    yp1 = int(image.shape[0] * y_point)
    yp2 = int(image.shape[0] * (y_point+0.05))
    crop = image[yp1:yp2, 0:xp]
    crop = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    params = cv2.SimpleBlobDetector_Params()
    params.filterByInertia = False
    params.filterByConvexity = False
    params.filterByColor = False
    params.minArea = minblobs
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(crop)
    blank = np.zeros((1, 1))
    blobs = cv2.drawKeypoints(crop, keypoints, blank, (0, 0, 255),
    cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return blobs, keypoints

def Find_slope(point_X, point_Y, y_1, y_2):
    global slope
    global intercept
    global xpointbot
    global xpointtop
    global xpointmid
    sum_1 = 0
    sum_2 = 0    
    y_mid = (y_1+y_2)/2
    x_bar = statistics.mean(point_X)
    y_bar = statistics.mean(point_Y)    
    for i in range(3):
        sum_1 = sum_1 + ((point_X[i]-x_bar) * (point_Y[i]-y_bar))
        sum_2 = sum_2 + ((point_X[i]-x_bar)**2)
    if sum_2 != 0:
        slope = sum_1/sum_2
    if slope != 0:
        intercept = y_bar - (slope*x_bar)
        xpointbot = (y_1-intercept)/slope
        xpointtop = (y_2-intercept)/slope
        xpointmid = (y_mid-intercept)/slope
    return xpointbot, xpointtop, xpointmid

def Detect_Curve(result):
    global x_C
    x_C[0], x_C[1], x_C[2] = Find_slope(blob_c, main_Y, y_bot, y_top,)   
    cv2.line(result, (int(x_C[2]), y_mid),(int(result.shape[1]*0.5), y_mid), (180, 0, 255), 1,  8)
    cv2.line(result, (int(x_C[0]), y_bot),(int(x_C[1]), y_top), (20,255,150), 2,  8)

def Cal_Servo(result,mid_point):
    Servo_morter = (mid_point/result.shape[1])*100
    return Servo_morter

def blobby(key_point,x):
    global blob_c
    if len(keypoint[x])==1:
        blob_c[x] = keypoint[x][0].pt[0]
    else :
        pass

def send_data():
    global MODE_COLOR

    rospy.init_node('send_data',anonymous=True)
    pub_angle = rospy.Publisher('angle',Float64, queue_size=10)
    pub_traffic = rospy.Publisher('/traffic',Int64, queue_size=10)
    sub_mode = rospy.Subscriber('Color_Mode',Int64,self.getlight, queue_size = 10)
    MODE_COLOR = Color_Mode.data

    pub_angle.publish(Servo_morter)
    pub_traffic.publish(run)
    #rospy.loginfo(Servo_morter)
    #rospy.loginfo(run)
    #rospy.loginfo(MODE_COLOR)



ocam = OCamInterface()
while True:
        frame = ocam.get_image()
        light = trafficgreen(frame)
        result,detect_color = Getcam(frame)
        #angle = send_angle()
        cv2.line(result, (int(result.shape[1]*0.5), y_bot),(int(result.shape[1]*0.5), y_top), (0, 0, 255), 1,  8)

        blobs[0], keypoint[0] = bloblane(detect_color, multiply_y[0],10)
        blobs[1], keypoint[1] = bloblane(detect_color, multiply_y[1],10)
        blobs[2], keypoint[2] = bloblane(detect_color, multiply_y[2],10)

        blobby(keypoint,0)
        blobby(keypoint,1)
        blobby(keypoint,2)
    
        BLOB=np.concatenate((blobs[2],blobs[1],blobs[0]),axis=0)
        Detect_Curve(result)
        Servo_morter = Cal_Servo(result,x_C[2])
        result =np.concatenate((result,BLOB),axis=0)
        angle = send_angle()
        image = trafficgreen(frame)
        send_data()

        if Servo_morter >100:
            Servo_morter=100
        elif Servo_morter<0:
            Servo_morter=0     
        else :
            pass
        detect_color  = cv2.resize(BLOB, (0,0), fx=0.5, fy=0.5) 
       # frame = cv2.resize(result, (0,0), fx=0.5, fy=0.5)
        result = cv2.resize(result, (0,0), fx=0.5, fy=0.5)  
       # detect_color=cv2.cvtColor(detect_color, cv2.COLOR_BGR2GRAY)
       # result=cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
       #print(x_C[2])
        #print(Servo_morter)
        cv2.imshow('frame',frame)
        cv2.imshow('roi',result)
        cv2.imshow('color',detect_color)
        cv2.imshow('traffic', image)

	 #cv2.imshow('light', light)
        
        if cv2.waitKey(1) & 0xff == ord('q'):
                break
cv2.destroyAllWindows()

def main(args):
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


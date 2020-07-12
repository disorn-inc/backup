import roslib
import sys
import rospy
import statistics
import cv2
import numpy as np
import statistics
import time
import math

ocam = OCamInterface()

lower_yellow = np.array([28, 100, 50])
upper_yellow = np.array([35, 232, 255])

lower_white = np.array([0,100,100])
upper_white = np.array([60,255,255])

global slope = 1
global intercept = 1
global xpointbot = 0
global xpointtop = 0
global xpointmid = 0

#open-camera
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
        rgb = cv2.resize(image,(0,0),fx=0.4,fy=0.4)
        return rgb

def Find_slope(point_X, point_Y, y_1, y_2):
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

def bloblane(image, y_point,minblobs):
    if minblobs == 0:
        minblobs  = image.shape[1]*0.08
    xp = int(image.shape[1])
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

while True:
        image = ocam.get_image()
        blob = bloblane(image)
        cv2.imshow('Detect Lane',blob)
        
        
        if cv2.waitKey(1) & 0xff == ord('q'):
               break
cv2.destroyAllWindows()
#!/usr/bin/env python
"""
OcamNode
for connecting with OCam
Author : TAI <punyapat.areerob@mail.kmutt.ac.th>
"""
import rospy
import cv2
import liboCams
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

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
        self.camera_control_list = None
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

        # Camera setting reassign (?)
        for i in range(len(self.camera_format_list)):
            self.camera.Set(self.camera_format_list[1])

        self.camera_name = self.camera.GetName()
        rospy.loginfo("Currently Connecting to Camera : " + str(self.camera_name))

        # Mute Data Printing if verbose is false
        if verbose_flag is False:
            self.camera.Close()
            self.camera = liboCams.oCams(
                self.camera_path,
                verbose=0
            )

        # Start Camera instance !
        self.camera.Start()

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

class ImageConverter(object):
    """Class for converting OpenCv's Mat into
    sensor_msgs/Image for ROS's Ecosystem
    """
    def __init__(self):
        self.image_pub = rospy.Publisher(
            "image",
            Image,
            queue_size=10
        )
        self.bridge = CvBridge()

    def publish(self, cv_image):
        """Publish OpenCV Mat in Sensor_msgs/Image Format

        Arguments:
            mat {OpenCV Mat} -- Matrix containing image Data
        """
        try:
            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(
                    cv_image,
                    "bgr8"
                )
            )
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    # Init ROS Node
    rospy.init_node("Ocam")
    ocam = OCamInterface()
    image_converter = ImageConverter()
    # Loop
    # rate = rospy.Rate(60) # 60hz
    while not rospy.is_shutdown():
        #rospy.loginfo("test")
        mat = ocam.get_image()
        image_converter.publish(mat)
        # rate.sleep()

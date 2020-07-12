#!/usr/bin/env python
import rospy
from std_msgs.msg import String ,Int8


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print(data.data)
def listener():

    rospy.init_node('sumtopic', anonymous=True)

    rospy.Subscriber("number", Int8, callback)
    rospy.Subscriber("text", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
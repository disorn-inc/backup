#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Leon Jung

import rospy
import numpy as np
import math
import tf
from enum import Enum
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def callback(x):    
    pass

class ControlParking():
    def __init__(self):

        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.cbOdom, queue_size=1)
        self.parking_start = rospy.Subscriber('/pak_or', UInt8, self.cbParkingStart, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.paking_order_st = rospy.Publisher('/pak_or_st', UInt8, queue_size = 1)

        self.theta = 0.0
        self.current_theta = 0.0
        self.last_current_theta = 0.0
        self.is_step_start = False
        self.current_step_of_parking = 1
        self.lastError = 0.0
        self.is_step_parking = True
        loop_rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
             if  self.is_step_parking == True:
                  self.fnParking()
            
                  loop_rate.sleep()

        rospy.on_shutdown(self.fnShutDown)

    def cbParkingStart(self, parking_start_msg):
        self.is_step_parking = True
        self.lastError = 0.0

    def fnParking(self):
        if self.current_step_of_parking == 1:
            rospy.loginfo("outer_turn_first")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta - 1.57
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.05:
                rospy.loginfo("outer_turn_first finished")
                self.current_step_of_parking = 2
                self.is_step_start = False

        elif self.current_step_of_parking == 2:
            rospy.loginfo("parking_lot_entry")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.start_pos_x = self.current_pos_x
                self.start_pos_y = self.current_pos_y
                self.is_step_start = True

            error = self.fnStraight(0.25)

            if math.fabs(error) < 0.005:
                rospy.loginfo("parking_lot_entry finished")
                self.current_step_of_parking = 3
                self.is_step_start = False            

        elif self.current_step_of_parking == 3:
            rospy.loginfo("parking_lot_turn_first")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta + 1.57
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.05:
                rospy.loginfo("parking_lot_turn_first finished")
                self.current_step_of_parking = 4
                self.is_step_start = False

        elif self.current_step_of_parking == 4:
            rospy.loginfo("parking_lot_stop")
            self.fnStop()

            rospy.sleep(2)

            rospy.loginfo("parking_lot_stop finished")
            self.current_step_of_parking = 5

        elif self.current_step_of_parking == 5:
            if self.is_step_start == False:
                rospy.loginfo("parking_lot_turn_second")
                self.lastError = 0.0
                self.desired_theta = self.current_theta + 1.57
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.05:
                rospy.loginfo("parking_lot_turn_second finished")
                self.current_step_of_parking = 6
                self.is_step_start = False

        elif self.current_step_of_parking == 6:
            rospy.loginfo("parking_lot_exit")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.start_pos_x = self.current_pos_x
                self.start_pos_y = self.current_pos_y
                self.is_step_start = True

            error = self.fnStraight(0.25)

            if math.fabs(error) < 0.005:
                rospy.loginfo("parking_lot_exit finished")
                self.current_step_of_parking = 7
                self.is_step_start = False   

        elif self.current_step_of_parking == 7:
            rospy.loginfo("outer_turn_second")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta - 1.57
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.05:
                rospy.loginfo("outer_turn_second finished")
                self.current_step_of_parking = 0
                self.is_step_start = False

        else:
            rospy.loginfo("idle (if finished to go out from parking lot)")
            msg_parking_finished = UInt8()
            msg_parking_finished.data = 1
            self.paking_order_st.publish(msg_parking_finished)
            self.fnStop()
            self.is_step_parking = False


    def cbOdom(self, odom_msg):
        quaternion = (odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w)
        self.current_theta = self.euler_from_quaternion(quaternion)

        if (self.current_theta - self.last_current_theta) < -math.pi:
            self.current_theta = 2. * math.pi + self.current_theta
            self.last_current_theta = math.pi
        elif (self.current_theta - self.last_current_theta) > math.pi:
            self.current_theta = -2. * math.pi + self.current_theta
            self.last_current_theta = -math.pi
        else:
            self.last_current_theta = self.current_theta

        self.current_pos_x = odom_msg.pose.pose.position.x
        self.current_pos_y = odom_msg.pose.pose.position.y
   
    def euler_from_quaternion(self, quaternion):
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]
        return theta
    def fnTurn(self):
        err_theta = self.current_theta - self.desired_theta
        
        rospy.loginfo("Parking_Turn")
        rospy.loginfo("err_theta  desired_theta  current_theta : %f  %f  %f", err_theta, self.desired_theta, self.current_theta)
        Kp = 0.8

        Kd = 0.03

        angular_z = Kp * err_theta + Kd * (err_theta - self.lastError)
        self.lastError = err_theta

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)

        rospy.loginfo("angular_z : %f", angular_z)

        return err_theta

    def fnStraight(self, desired_dist):
        err_pos = math.sqrt((self.current_pos_x - self.start_pos_x) ** 2 + (self.current_pos_y - self.start_pos_y) ** 2) - desired_dist
        
        rospy.loginfo("Parking_Straight")

        Kp = 0.4
        Kd = 0.05

        self.lastError = err_pos

        twist = Twist()
        twist.linear.x = 0.07
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

        return err_pos

    def fnStop(self):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)


    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist) 

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('control_parking')
    node = ControlParking()
    node.main()

#!/usr/bin/env python
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
        self.sub_scan_obstacle = rospy.Subscriber('/scan', LaserScan, self.cbScanObstacle, queue_size=1)
        self.theta = 0.0
        self.current_theta = 0.0
        self.last_current_theta = 0.0
        self.is_step_start = False
        self.current_step_of_parking = 1
        self.lastError = 0.0
        self.is_step_parking = True
        self.side = 0
        self.realside = 0
        loop_rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
             if  self.is_step_parking == True:
                  self.fnParking()
            
                  loop_rate.sleep()

        rospy.on_shutdown(self.fnShutDown)
    def cbScanObstacle(self, scan):
        self.side = 0
        for i in range(80,100):
            if scan.ranges[i] < 0.5 and scan.ranges[i] > 0.01:
                self.side = 2
        for i in range(260,280):
            if scan.ranges[i] < 0.5 and scan.ranges[i] > 0.01:
                self.side = 1
    def cbParkingStart(self, parking_start_msg):
        self.is_step_parking = True
        self.lastError = 0.0

    def fnParking(self):
        print(self.current_step_of_parking)
        if self.current_step_of_parking == 1:
            rospy.loginfo("find anortor car")
            if self.side == 1:
                self.current_step_of_parking = 2
                self.realside = 1
            elif self.side == 2:
                self.current_step_of_parking = 3
                self.realside = 2


        elif self.current_step_of_parking == 2:
            rospy.loginfo("Left")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta + 1.57
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.05:
                rospy.loginfo("Left finished")
                self.current_step_of_parking = 4
                self.is_step_start = False         


        elif self.current_step_of_parking == 3:
            rospy.loginfo("Right")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta - 1.57
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.05:
                rospy.loginfo("Right finished")
                self.current_step_of_parking = 4
                self.is_step_start = False


        elif self.current_step_of_parking == 4:
            rospy.loginfo("in_parking")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.start_pos_x = self.current_pos_x
                self.start_pos_y = self.current_pos_y
                self.is_step_start = True

            error = self.fnStraight(0.25)

            if math.fabs(error) < 0.005:
                rospy.loginfo("parking_lot_in finished")
                self.current_step_of_parking = 5
                self.is_step_start = False   


        elif self.current_step_of_parking == 5:
            rospy.loginfo("parking_lot_stop")
            self.fnStop()

            rospy.sleep(2)

            rospy.loginfo("parking_lot_stop finished")
            self.current_step_of_parking = 6


        elif self.current_step_of_parking == 6:
            rospy.loginfo("Turn around")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta + 3.14
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.05:
                rospy.loginfo("Turn finished")
                self.current_step_of_parking = 7
                self.is_step_start = False


        elif self.current_step_of_parking == 7:
            rospy.loginfo("out_parking")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.start_pos_x = self.current_pos_x
                self.start_pos_y = self.current_pos_y
                self.is_step_start = True

            error = self.fnStraight(0.25)

            if math.fabs(error) < 0.005:
                rospy.loginfo("parking_lot_in finished")
                if self.realside==1:
                    self.current_step_of_parking = 9
                if self.realside==2:
                    self.current_step_of_parking = 8
                self.is_step_start = False   
        

        elif self.current_step_of_parking == 8:
            rospy.loginfo("Right")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta - 1.57
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.05:
                rospy.loginfo("Right finished")
                self.current_step_of_parking = 10
                self.is_step_start = False

        elif self.current_step_of_parking == 9:
            rospy.loginfo("Left")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.desired_theta = self.current_theta + 1.57
                self.is_step_start = True

            error = self.fnTurn()

            if math.fabs(error) < 0.05:
                rospy.loginfo("Left finished")
                self.current_step_of_parking = 10
                self.is_step_start = False


        elif self.current_step_of_parking == 10:
            rospy.loginfo("in_parking")
            if self.is_step_start == False:
                self.lastError = 0.0
                self.start_pos_x = self.current_pos_x
                self.start_pos_y = self.current_pos_y
                self.is_step_start = True

            error = self.fnStraight(0.25)

            if math.fabs(error) < 0.005:
                rospy.loginfo("parking_lot_in finished")
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

        angular_z = Kp * err_pos + Kd * (err_pos - self.lastError)
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

#!/usr/bin/env python

import rospy
import math
import numpy as np
import time

from classes.variables import *
from classes.imu_publisher import *
from classes.PID import *

from robot_micro.cfg import controllerConfig

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server

rad2degrees = 180.0/math.pi


class RobotControllerLQRNode:
    def __init__(self):
        """ Init a new robot controller """
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.offset_roll = 0.0
        self.output = 0
        self.desired_x = 0
        self.desired_z = 0
        self.current_position_x = 0
        self.derivative_position_x = 0
        self.twist = Twist()

        self.angular_x = 0

        self.current_time = time.time()
        self.last_time = self.current_time

        self.constant_scaling = 1/600

        """Parameters for calibration"""
        self.total_frames_calibration = 40
        self.actual_frame_calibration = 0
        self.offset_position_x = 0
        self.offset_vector = np.zeros((self.total_frames_calibration, 1))
        self.scaling_joy = 1.0

        """Setup ROS Publisher/Subscribers"""
        self.sub = rospy.Subscriber('/robot/imu', Imu, self.process_imu_message)
        self.sub_twist = rospy.Subscriber('/robot/cmd_vel', Twist, self.process_twist_message)
        self.sub_odometry = rospy.Subscriber('/robot/odom', Odometry, self.process_odometry_message)

        # publish to cmd_vel
        self.pub_vel = rospy.Publisher('/robot/cmd_vel_real', Twist, queue_size=1)

        rospy.spin()

    def process_imu_message(self, imuMsg):
        quaternion = (
            imuMsg.orientation.x,
            imuMsg.orientation.y,
            imuMsg.orientation.z,
            imuMsg.orientation.w)

        self.angular_x = imuMsg.angular_velocity.x

        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion)

        self.roll = self.roll * rad2degrees
        self.pitch = self.pitch * rad2degrees
        self.yaw = self.yaw * rad2degrees

        if self.roll > 180.0:
            self.roll -= 360.0
        if self.roll < -180.0:
            self.roll += 360.0

        # rospy.loginfo("Angle: roll %f pitch %f yaw %f", self.roll, self.pitch, self.yaw)

        '''First measurements used for calibrate initial position robot'''
        if self.actual_frame_calibration < self.total_frames_calibration:
            self.calibrate_controller()
            self.actual_frame_calibration += 1
        else:
            self.offset_roll = np.mean(self.offset_vector)
            self.update_controller()

    def process_odometry_message(self, odometry_msg):
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        self.current_position_x = odometry_msg.pose.pose.position.x
        self.derivative_position_x = delta_time * self.current_position_x
        self.last_time = self.current_time

    def process_twist_message(self, twist_msg):
        self.desired_x = twist_msg.linear.x
        self.desired_z = twist_msg.angular.z

        rospy.loginfo("changed: desired_x: %f self.desired_z: %f ", self.desired_x, self.desired_z )

    def calibrate_controller(self):
        self.offset_vector[self.actual_frame_calibration] = self.roll

    def update_controller(self):

        lqr_1 = -4.4721 * (self.current_position_x - self.desired_x)
        lqr_2 = 272.7668 * self.roll
        lqr_3 = -12.9472 * self.derivative_position_x
        lqr_4 = 59.4279 * self.angular_x

        self.output = - self.constant_scaling * (lqr_1 + lqr_2 + lqr_3 + lqr_4)

        #self.output = self.controller_position.output
        # create a twist message, fill in the details
        #self.twist.linear.x = - self.output*( 1 + self.desired_x)
        self.twist.linear.x = self.output
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = self.desired_z

        self.pub_vel.publish(self.twist)

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('robot_imu_controller')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        asd = RobotControllerLQRNode()
    except rospy.ROSInterruptException:
        pass
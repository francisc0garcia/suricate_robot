#!/usr/bin/env python
'''
Copyright (C) 2016 Francisco Garcia

This program is free software; you can redistribute it and/or modify it under the terms of the
GNU General Public License as published by the Free Software Foundation; either version 2 of the License,
or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.
'''

import rospy
import math
from math import pi, asin, acos
import numpy as np
import time
from threading import Timer,Thread,Event

from classes.variables import *
from classes.imu_publisher import *
from classes.PID import *
from classes.linear_kalman_filer import *
from classes.ControllerPosition import *
from classes.PoseRobot import *

from suricate_robot.cfg import controllerConfig

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server

rad2degrees = 180.0/math.pi

class RobotControllerNode:
    def __init__(self):
        # publish to cmd_vel
        self.pub_vel = rospy.Publisher('/robot/cmd_vel_real', Twist, queue_size=1)

        # Reset output
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        self.pub_vel.publish(self.twist)

        """ Init a new robot controller """
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.offset_roll = 0.0
        self.offset_pitch = 0.0
        self.output = 0.0
        self.desired_x = 0.0
        self.desired_z = 0.0
        self.current_position_x = 0.0
        self.current_vel_x = 0.0

        self.enable_controller = False

        self.pose = PoseRobot()
        self.initPose()
        self.goal = None

        self.dx = 0
        self.dr = 0
        self.gz = 0
        self.Idx = 0
        self.Idr = 0

        self.scaling_joy = 1.0

        """initialize controllers"""
        # kalman filter setup
        self.kalman_joy_A = np.matrix([1.25])
        self.kalman_joy_H = np.matrix([1])
        self.kalman_joy_B = np.matrix([0])
        self.kalman_joy_Q = np.matrix([0.00001])
        self.kalman_joy_R = np.matrix([0.1])
        self.kalman_joy_xhat = np.matrix([0])
        self.kalman_joy_P    = np.matrix([1])
        self.kalman_joy_filter = KalmanFilterLinear(self.kalman_joy_A,self.kalman_joy_B,
                                                    self.kalman_joy_H,self.kalman_joy_xhat,
                                                    self.kalman_joy_P,self.kalman_joy_Q,self.kalman_joy_R)
        rospy.on_shutdown(self.shutdown_node)

        """Setup ROS Publisher/Subscribers"""
        self.sub = rospy.Subscriber('/robot/imu', Imu, self.process_imu_message, queue_size=1)
        self.sub_twist = rospy.Subscriber('/robot/cmd_vel', Twist, self.process_twist_message, queue_size=1)
        self.sub_odometry = rospy.Subscriber('/robot/odom', Odometry, self.process_odometry_message, queue_size=1)
        #self.sub_desired_odometry = rospy.Subscriber('/robot/desired_odom', Odometry, self.process_desired_odometry_message, queue_size=1)

        self.srv = Server(controllerConfig, self.reconfig_callback) # define dynamic_reconfigure callback

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            self.update_controller()
            rate.sleep()

        #rospy.spin()

    def initPose(self):
        self.pose = PoseRobot()
        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = 0

    def process_imu_message(self, imuMsg):
        quaternion = (
            imuMsg.orientation.x,
            imuMsg.orientation.y,
            imuMsg.orientation.z,
            imuMsg.orientation.w)

        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion)

        self.roll = self.roll * rad2degrees  + self.offset_roll
        self.pitch = self.pitch * rad2degrees + self.offset_pitch
        self.yaw = self.yaw * rad2degrees

        self.gz = imuMsg.angular_velocity.x

        #rospy.loginfo("Angle: roll %f pitch %f yaw %f", self.roll, self.pitch, self.yaw)
        #time.sleep(0.2)

        #test!
        '''
        if (self.roll - 2.25) * imuMsg.orientation.y > 0 :
            self.roll = imuMsg.orientation.y * rad2degrees
        else:
            self.roll = -imuMsg.orientation.y * rad2degrees
        '''
        # self.update_controller()

    def process_odometry_message(self, odometry_msg):
        self.current_position_x = odometry_msg.pose.pose.position.x
        self.current_vel_x = odometry_msg.twist.twist.linear.x

        self.dx = odometry_msg.twist.twist.linear.x
        self.dr = odometry_msg.twist.twist.angular.z

        self.pose = PoseRobot()
        pos = odometry_msg.pose.pose.position
        orientation = odometry_msg.pose.pose.orientation
        self.pose.x = pos.x
        self.pose.y = pos.y
        self.pose.theta = 2 * acos(orientation.w)

    def process_twist_message(self, twist_msg):
        if twist_msg.linear.x > 0:
            self.enable_controller = True

        self.desired_x = self.kalman_joy_filter.GetCurrentState()[0,0] * self.scaling_joy * 0.9
        self.desired_z = twist_msg.angular.z

        self.kalman_joy_filter.Step(np.matrix([0]), np.matrix([twist_msg.linear.x]))

        self.desired_x = self.bound_limit(self.desired_x, -30.0, 30.0)

        # rospy.loginfo("desired_x: %f measure: %f", self.desired_x, twist_msg.linear.x )
        # rospy.loginfo("changed: desired_x: %f self.desired_z: %f ", self.desired_x, self.desired_z )

    def reconfig_callback(self, config, level):
        self.scaling_joy = config['scaling_joy']

        return config

    def update_controller(self):
        # Update integral
        self.Idx = self.Idx + (self.dx - self.Idx)/2
        self.Idr = self.Idr + (self.dr - self.Idr)/2

        # setup up sampling time at 10 hz
        v_xi = self.Idx      # integral self.dx
        v_psi = self.Idr     # integral self.dr
        v_phi = self.pitch   # pitch
        dv_xi = self.dx      # self.dx
        dv_psi = self.dr     # self.dr
        dv_phi = self.gz     # gy ? or gz

        x = np.array( [v_xi, v_psi, v_phi, dv_xi, dv_psi, dv_phi])
        x_transposed = np.transpose(x)

        Kdr = np.array(  [(-1.5127, -0, -124.013, -4.5334,  0, -36.1285),
                          (0, 1.6286, 0, 0, 2.6639, 0) ] )

        u = np.dot(-Kdr,  x_transposed)     # u = [linear.x; angular.z]

        # for debugging:
        #print Kdr
        #print x_transposed
        #print u
        #rospy.loginfo("u: 1: %f 2: %f", u[0], u[1])

        self.twist.linear.x =  -self.bound_limit(u[0], -1, 1)
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0 # -self.bound_limit(u[1], -1, 1)

        self.pub_vel.publish(self.twist)

    def disable_controller(self):
        self.enable_controller = False

    def bound_limit(self, n, minn, maxn):
        return max(min(maxn, n), minn)

    def send_reset_command(self):
        # create a twist message, fill in the details
        self.twist.linear.x = self.output
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = self.desired_z * self.scaling_joy

        self.pub_vel.publish(self.twist)

    def shutdown_node(self):
        rospy.loginfo("Turning off node: robot_controller")
        self.send_reset_command()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('robot_imu_controller')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        asd = RobotControllerNode()
    except:

        pass
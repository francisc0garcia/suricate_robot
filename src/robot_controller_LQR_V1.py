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

from suricate_robot.cfg import controllerConfig

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Wrench
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server

rad2degrees = 180.0/math.pi

class RobotControllerNode:
    def __init__(self):
        # Setup ROS Publisher
        self.pub_wrench = rospy.Publisher('/robot/cmd_wrench', Wrench, queue_size=1)
        self.pub_angle = rospy.Publisher('/robot/angle', Float32, queue_size=1)
        self.pub_path = rospy.Publisher('/robot/path', Float32, queue_size=1)

        # Reset output wrench
        self.wrench_cmd = Wrench()
        self.pub_wrench.publish(self.wrench_cmd)

        self.angle_msg = Float32()
        self.angle_msg.data = 0
        self.pub_angle.publish(self.angle_msg)

        self.path_msg = Float32()
        self.path_msg.data = 0
        self.pub_path.publish(self.path_msg)

        # Init values
        [self.roll, self.pitch, self.yaw, self.offset_roll, self.offset_pitch] = [0.0, 0.0, 0.0, 0.0, 0.0]
        [self.output, self.desired_x, self.desired_z, self.current_position_x, self.current_vel_x] = [0.0, 0.0, 0.0, 0.0, 0.0]
        [self.dx, self.dr, self.gz, self.Idx, self.Idr] = [0.0, 0.0, 0.0, 0.0, 0.0]
        [self.xi_des, self.psi_des] = [0.0, 0.0]
        self.IIdx = 0.0

        self.enable_controller = False
        self.scaling_joy = 1.0

        rospy.on_shutdown(self.shutdown_node)

        # Read input parameters from launch file
        self.rate = rospy.get_param('~rate', 200.0)
        self.gain_LQR = rospy.get_param('~gain_LQR', 0.007)

        # Setup ROS Subscribers
        self.sub = rospy.Subscriber('/robot/imu', Imu, self.process_imu_message, queue_size=1)
        self.sub_twist = rospy.Subscriber('/robot/cmd_vel', Twist, self.process_twist_message, queue_size=1)
        self.sub_odometry = rospy.Subscriber('/robot/odom', Odometry, self.process_odometry_message, queue_size=1)

        self.srv = Server(controllerConfig, self.reconfig_callback) # define dynamic_reconfigure callback

        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            self.update_controller()
            rate.sleep()

    def process_imu_message(self, imuMsg):
        quaternion = (
            imuMsg.orientation.x,
            imuMsg.orientation.y,
            imuMsg.orientation.z,
            imuMsg.orientation.w)

        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion)

        # Convert to degrees and add offset
        self.roll = self.roll  + self.offset_roll
        self.pitch = self.pitch   + self.offset_pitch
        self.yaw = self.yaw

        self.gz = imuMsg.angular_velocity.x

        #rospy.loginfo("Angle: roll %f pitch %f yaw %f", self.roll, self.pitch, self.yaw)
        #time.sleep(0.2)

    def process_odometry_message(self, odometry_msg):
        self.current_position_x = odometry_msg.pose.pose.position.x
        self.current_vel_x = odometry_msg.twist.twist.linear.x

        self.dx = odometry_msg.twist.twist.linear.x
        self.dr = odometry_msg.twist.twist.angular.z

        # Update integral
        # self.Idx = self.Idx + self.dx
        # self.Idr = self.Idr + self.dr

        quaternion = (
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w)


        self.Idx = self.current_position_x #this is in inertial frame!
        (tmDummy, tmpDummy, self.Idr) = euler_from_quaternion(quaternion)

    def process_twist_message(self, twist_msg):
        if twist_msg.linear.x > 0:
            self.enable_controller = True

        self.desired_x = twist_msg.linear.x
        self.desired_z = twist_msg.angular.z

    def reconfig_callback(self, config, level):
        self.scaling_joy = config['scaling_joy']

        return config

    def update_controller(self):
        # setup up sampling time at 10 hz
        v_xi = self.Idx      # integral self.dx from encoder (relative theta * r_w)
        v_psi = self.Idr     # integral self.dr
        v_phi = self.pitch - 0.0   # pitch
        dv_xi = self.dx      # self.dx from encoder (relative theta * r_w)
        dv_psi = self.dr     # self.dr
        dv_phi = self.gz     # gy ? or gz


        # correction of the angle (only for robot)
        #r_w = 0.273
        #v_xi = v_xi - v_phi*r_w
        #dv_xi = dv_xi - dv_phi*r_w
        #self.Idx = self.Idx + dv_xi / 150
        #v_xi = self.Idx / 1.8 # correction factor
        #self.IIdx = self.IIdx + v_xi / 150

        # TODO: do something like: if a move-command comes, set xi_des = xi_des + epsilon

        self.xi_des = 0
        self.psi_des = 0

        x = np.array( [v_xi - self.xi_des , v_psi - self.psi_des, v_phi, dv_xi, dv_psi, dv_phi])
        x_transposed = np.transpose(x)

        #k_dr = np.array(  [(-1.05, -0, -160.72 , -4.4415,  0, -40.751), # Ts  = 0.1
         #                 (0, 1.6286, 0, 0, 2.6639, 0) ] )
        #k_dr = np.array(  [(-1.7321/1000 , -0, -170.77, -6.756/1000 ,  0, 39.551), # cont
         #             (0, 1.6286, 0, 0, 2.6639, 0) ] )
        #k_dr = np.array(  [(  -2.27/20 , -0, -155.1, -6.6/70,  0, 37.9), # cont
         #                                (0, -0.3088, 0, 0, -0.55, 0) ] )
        k_dr = np.array(  [(  -1.4*11.5 , -0, -194.5*1.5, -5.82*7,  0, -49.3/1.9), # cont for gazebo
                         (0, -0.3088, 0, 0, -0.55, 0) ] )


        u = np.dot(k_dr,  x_transposed)     # u = [linear.x; angular.z]

        # for debugging:
        # rospy.loginfo("v_xi: %f v_psi: %f v_phi: %f dv_xi: %f dv_psi: %f dv_phi: %f", v_xi, v_psi, v_phi, dv_xi, dv_psi, dv_phi)

        #[0] = u[0] + 0.001 * self.IIdx

        #====================================
        # compensate for the dead zone
        #if np.abs(u[0]) < 2.5 :
         #   if np.abs(u[0]) < 0.1:
          #      u[0] = 0.0
           # else :
            #    u[0] = 2.5 *np.sign(u[0])
        #====================================

        #self.gain_LQR =  0.0019 # for the robot
        self.gain_LQR =  0.05 # for gazebo
        # Publish new wrench value
        self.wrench_cmd.force.x = self.bound_limit(u[0] * self.gain_LQR, -2, 2)
        self.wrench_cmd.force.y = 0
        self.wrench_cmd.force.z = 0
        self.wrench_cmd.torque.x = 0
        self.wrench_cmd.torque.y = 0 # self.bound_limit(u[0]*0.2, -10, 10)
        self.wrench_cmd.torque.z = self.bound_limit(u[1] * self.gain_LQR * 1, -2, 2)
        self.pub_wrench.publish(self.wrench_cmd)

        self.angle_msg.data = dv_phi #* rad2degrees
        self.pub_angle.publish(self.angle_msg )
        self.path_msg.data = dv_xi
        self.pub_path.publish(self.path_msg)

    def disable_controller(self):
        self.enable_controller = False

    def bound_limit(self, n, minn, maxn):
        return max(min(maxn, n), minn)

    def send_reset_command(self):
        self.wrench_cmd.force.x = 0
        self.wrench_cmd.force.y = 0
        self.wrench_cmd.force.z = 0
        self.wrench_cmd.torque.x = 0
        self.wrench_cmd.torque.y = 0
        self.wrench_cmd.torque.z = 0
        self.pub_wrench.publish(self.wrench_cmd)

    def shutdown_node(self):
        rospy.loginfo("Turning off node: robot_controller")
        self.send_reset_command()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('robot_imu_controller')

    try:
        temp = RobotControllerNode()
    except:
        pass
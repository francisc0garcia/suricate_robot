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
import numpy as np
import math
import smbus
import time

from classes.variables import *

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
from visualization_msgs.msg import Marker

from classes.HT16K33.EightByEight import *

rad2degrees = 180.0/math.pi

# define the method for generating a one dimensional MultiArray
def make_multi_array(iterable, label):
    array_list = []
    for el in iterable:
        array_list.append(el)
    dim = MultiArrayDimension()
    dim.size = len(array_list)
    dim.label = label
    dim.stride = len(array_list)

    temp_array = Int16MultiArray()
    temp_array.data = array_list
    temp_array.layout.dim.append(dim)
    temp_array.layout.data_offset = 0
    return temp_array

class RobotSafetyChecker:
    def __init__(self):
        """Init parameters"""
        '''
        List of system variables:
        0. Arduino connection
        1. IMU
        '''
        self.list_system_variables = np.array([0, 0])

        '''Define variables'''
        self.battery_level = 0
        self.battery_level_msg = Float32()
        self.battery_level_msg.data = self.battery_level

        ''' IMU data '''
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.imu_max_y = 80

        self.imu_quat_x = 0.0
        self.imu_quat_y = 0.0
        self.imu_quat_z = 0.0
        self.imu_quat_w = 0.0

        '''Odometry'''
        self.current_position_x = 0.0
        self.current_position_y = 0.0
        self.current_position_max = 2000

        '''Joy - command velocity'''
        self.desired_linear_vel_x = 0.0
        self.desired_angular_vel_z = 0.0
        self.desired_max_x = 1
        self.desired_max_z = 1
        self.actual_command = 'stop'

        '''I2C Arduino'''
        self.bus = smbus.SMBus(1)
        self.bytes = []
        self.address_arduino = 0x04
        self.bytes_from_arduino = range(5)

        '''LED matrix'''
        self.matrix = EightByEight(address=0x70, bus=self.bus ).setUp()
        self.matrix.setBrightness(10)
        self.matrix.clear()

        self.matrix.draw_matrix( self.matrix.symbols.tu_logo() )
        time.sleep(1)

        self.rate = rospy.get_param('~rate', 100.0)  # the rate at which to publish the transform

        # publish to cmd_vel
        self.pub_sys_state = rospy.Publisher('/robot/system_state', Int16MultiArray, queue_size=1)
        self.pub_bat = rospy.Publisher('/robot/battery', Float32, queue_size=1)
        self.marker_robot_pub = rospy.Publisher("/robot/marker_position", Marker, queue_size=1)

        """Setup ROS Publisher/Subscribers"""
        self.sub = rospy.Subscriber('/robot/imu', Imu, self.process_imu_message, queue_size=1)
        self.sub_twist = rospy.Subscriber('/robot/cmd_vel', Twist, self.process_twist_message, queue_size=1)
        self.sub_odometry = rospy.Subscriber('/robot/odom', Odometry, self.process_odometry_message, queue_size=1)

        rospy.on_shutdown(self.shutdown_node)

        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            self.publish_system_state()
            rate.sleep()

    def demo_matrix(self):
        self.matrix.draw_matrix( self.matrix.symbols.sad_face() )
        time.sleep(2)

        self.matrix.draw_matrix( self.matrix.symbols.question_mark() )
        time.sleep(2)

        self.matrix.draw_matrix( self.matrix.symbols.tu_logo() )
        time.sleep(2)

    def process_imu_message(self, imuMsg):
        self.imu_quat_x = imuMsg.orientation.x
        self.imu_quat_y = imuMsg.orientation.y
        self.imu_quat_z = imuMsg.orientation.z
        self.imu_quat_w = imuMsg.orientation.w

        quaternion = (
            self.imu_quat_x,
            self.imu_quat_y,
            self.imu_quat_z,
            self.imu_quat_w)

        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion)

        self.roll = self.roll * rad2degrees
        self.pitch = self.pitch * rad2degrees
        self.yaw = self.yaw * rad2degrees

        if abs(self.pitch) > self.imu_max_y:
            self.list_system_variables[1] = 0
            # rospy.loginfo("IMU orientation reach the limit: %f of %f ", abs(imuMsg.orientation.y), self.imu_max_y )
        else:
            self.list_system_variables[1] = 1

        self.publish_system_state()

    def process_odometry_message(self, odometry_msg):
        self.current_position_x = odometry_msg.pose.pose.position.x
        self.current_position_y = odometry_msg.pose.pose.position.y

    def process_twist_message(self, twist_msg):
        self.desired_linear_vel_x = twist_msg.linear.x
        self.desired_angular_vel_z = twist_msg.angular.z

        self.actual_command = 'stop'

        if self.desired_linear_vel_x > 0:
            self.actual_command = 'up'

        if self.desired_linear_vel_x < 0:
            self.actual_command = 'down'

        if self.desired_angular_vel_z > 0:
            self.actual_command = 'left'

        if self.desired_angular_vel_z < 0:
            self.actual_command = 'right'

    def publish_system_state(self):
        array_pub = make_multi_array(self.list_system_variables, "system_state")
        self.pub_sys_state.publish(array_pub)

        battery_level = self.read_battery_status()

        if battery_level > 0:
            self.publish_battery_level()
            self.list_system_variables[0] = 1
        else:
            self.list_system_variables[0] = 0

        self.publish_marker_position()
        self.update_led_matrix()

        # self.demo_matrix()

    def update_led_matrix(self):
        if self.list_system_variables[0] == 0:
            self.matrix.draw_matrix( self.matrix.symbols.question_mark() )

        if self.list_system_variables[1] == 0:
            self.matrix.draw_matrix( self.matrix.symbols.sad_face() )

        if self.list_system_variables[0] == 1 and self.list_system_variables[1] == 1:

            if self.actual_command == 'stop':
                self.matrix.draw_matrix( self.matrix.symbols.happy_face() )
            elif self.actual_command == 'up':
                self.matrix.draw_matrix( self.matrix.symbols.arrow_up() )
            elif self.actual_command == 'down':
                self.matrix.draw_matrix( self.matrix.symbols.arrow_down() )
            elif self.actual_command == 'left':
                self.matrix.draw_matrix( self.matrix.symbols.arrow_left() )
            elif self.actual_command == 'right':
                self.matrix.draw_matrix( self.matrix.symbols.arrow_right() )
            else:
                self.matrix.draw_matrix( self.matrix.symbols.question_mark() )

        time.sleep(0.01)

    def read_battery_status(self):
        try:
            # send imu data
            self.bus.write_byte_data(self.address_arduino, 0x00, self.list_system_variables[1] )

            self.bytes_from_arduino[0] = self.bus.read_byte(self.address_arduino)
            self.battery_level = self.bytes_from_arduino[0]
            return  self.battery_level
        except IOError:
            self.battery_level = -1
            return -1
        except Exception:
            self.battery_level = -2
            return -2
            # pass

    def publish_battery_level(self):
        self.battery_level_msg.data = self.battery_level
        self.pub_bat.publish(self.battery_level_msg)

    def publish_marker_position(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.CUBE
        marker.action = marker.MODIFY
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = self.current_position_x
        marker.pose.position.y = self.current_position_y
        marker.pose.position.z = 0.25

        marker.pose.orientation.x = -self.imu_quat_x
        marker.pose.orientation.y = -self.imu_quat_y
        marker.pose.orientation.z = self.imu_quat_z
        marker.pose.orientation.w = -self.imu_quat_w

        self.marker_robot_pub.publish(marker)

    def shutdown_node(self):
        rospy.loginfo("Turning off node: robot_safety_checker")

        self.matrix.draw_matrix( self.matrix.symbols.turning_off() )
        time.sleep(1)

        self.list_system_variables[0] = 0
        self.list_system_variables[1] = 0
        array_pub = make_multi_array(self.list_system_variables, "system_state")
        self.pub_sys_state.publish(array_pub)

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('robot_imu_controller')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        object_checker = RobotSafetyChecker()

    except rospy.ROSInterruptException:
        pass
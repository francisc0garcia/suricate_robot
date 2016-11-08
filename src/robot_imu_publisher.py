#!/usr/bin/env python

# Import ROS
import rospy
from geometry_msgs.msg import Twist
import time
import math
import numpy as np

import pigpio

# import libraries
from classes.variables import *
from classes.imu_publisher import *
from classes.OdometryPublisher import *

class RobotImuPublisherNode:
    def __init__(self):
        # create object to store robot data
        self.robot_data = RobotVariables()

        self.offset_pitch = rospy.get_param('~offset_pitch', 0)
        self.offset_roll = rospy.get_param('~offset_roll', 0)
        self.offset_yaw = rospy.get_param('~offset_yaw', 0)
        self.rate = rospy.get_param('~rate', 100.0)  # the rate at which to publish the transform

        self.gravity_offset = -1

        self.calibration_file = 'calibration.json'
        rospy.loginfo("IMU offset pitch: %d roll: %d yaw: %d",  self.offset_pitch, self.offset_roll, self.offset_yaw)

        # IMU variables
        self.bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=17)

        self.init_imu()

        # sw, bl, accel, mag, gyro = self.bno.get_revision()

        # Create a publisher for imu message
        self.pub_imu = rospy.Publisher('/robot/imu', Imu, queue_size=1)
        self.imu_publisher = ImuPublisher()

        self.current_time = rospy.get_time()
        self.last_time = rospy.get_time()

        self.seq = 0

        # Odometry
        self.robot_data.velocity_x = 0.0
        self.robot_data.velocity_y = 0.0
        self.robot_data.velocity_theta = 0

        rospy.on_shutdown(self.shutdown_node)

        rate = rospy.Rate(self.rate)

        [yaw, roll, pitch, lx, ly, lz, gx, gy, gz] = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Main while loop.
        while not rospy.is_shutdown():
            self.current_time = rospy.get_time()

            try:
                # qx, qy, qz, qw = self.bno.read_quaterion()
                # mx, my, mz = self.bno.read_magnetometer()
                # ax, ay, az = self.bno.read_accelerometer()
                # gvx, gvy, gvz = self.bno.read_gravity()
                yaw, roll, pitch = self.bno.read_euler()
                gx, gy, gz = self.bno.read_gyroscope()
                # lx, ly, lz = self.bno.read_linear_acceleration()
                # rospy.loginfo("read_gravity gvx: %d gvy: %d gvz: %d",  gvx, gvy, gvz)
            except:
                rospy.loginfo("Not possible to publish imu data")
                #[yaw, roll, pitch] = [0, 0, 0]
                self.init_imu(is_init_imu = False, is_init_device = True, is_init_calibration = False)
                #time.sleep(1)

            if abs(yaw) > 0 or abs(roll) > 0 or abs(pitch) > 0:
                # IMU info
                self.robot_data.imu_linear_acceleration_x = lx
                self.robot_data.imu_linear_acceleration_y = ly
                self.robot_data.imu_linear_acceleration_z = lz
                self.robot_data.imu_angular_velocity_x = gx
                self.robot_data.imu_angular_velocity_y = gy
                self.robot_data.imu_angular_velocity_z = gz
                self.robot_data.imu_euler_yaw = yaw + self.offset_yaw
                self.robot_data.imu_euler_roll = roll + self.offset_roll
                self.robot_data.imu_euler_pitch = pitch + self.offset_pitch
                # rospy.loginfo("roll %f pitch  %f  yaw  %f ", roll, pitch, yaw)

                # publish information over ROS
                self.imu_publisher.publish_info(self.pub_imu, self.robot_data)
                self.imu_publisher.set_sequence(self.seq)

                self.seq += 1

            rate.sleep()

    def init_imu(self, is_init_imu = False, is_init_device = False, is_init_calibration = False):

        while not is_init_imu:
            try:
                self.bno.begin()
                is_init_imu = True
            except:
                rospy.loginfo('Failed to initialize BNO055! trying again...')
                time.sleep(0.1)

        while not is_init_device :
            status, self_test, error = self.bno.get_system_status(False)
            if error == 0:
                is_init_device = True
            else:
                rospy.loginfo('Failed to initialize get_system_status error!')
                time.sleep(0.1)

        if not is_init_calibration:
            # Load precomputed calibration values
            self.load_calibration()

        while not is_init_calibration:
            cal_sys, cal_gyro, cal_accel, cal_mag = self.bno.get_calibration_status()

            if cal_gyro > 0 and cal_accel > 0:
                is_init_calibration = True
            else:
                self.load_calibration()
                #time.sleep(0.1)
                #rospy.loginfo("Waiting for IMU calibration: [S %f, G %f, A %f, M %f]" % (cal_sys, cal_gyro, cal_accel, cal_mag))
                time.sleep(0.1)

    def load_calibration(self):
        # computed using tutorial:
        # https://learn.adafruit.com/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black/webgl-example
        data = [253, 255, 5, 0, 166, 0, 205, 246, 93, 252, 95, 1, 254, 255, 255, 255, 1, 0, 232, 3, 163, 1]
        self.bno.set_calibration(data)
        return 'OK'

    def shutdown_node(self):
        rospy.loginfo("Turning off node: robot_imu_publisher")

        # IMU info
        self.robot_data.imu_linear_acceleration_x = 0
        self.robot_data.imu_linear_acceleration_y = 0
        self.robot_data.imu_linear_acceleration_z = 0
        self.robot_data.imu_angular_velocity_x = 0
        self.robot_data.imu_angular_velocity_y = 0
        self.robot_data.imu_angular_velocity_z = 0
        self.robot_data.imu_euler_yaw = 0
        self.robot_data.imu_euler_roll = 0
        self.robot_data.imu_euler_pitch = 0

        # publish information over ROS
        self.imu_publisher.publish_info(self.pub_imu, self.robot_data)

# Main function.
if __name__ == '__main__':
    from Adafruit_BNO055 import BNO055

    pi = pigpio.pi()

    while not pi.connected:
        rospy.loginfo('Failed to initialize pigpio. Trying again...')
        time.sleep(1)
        pi = pigpio.pi()

    # Initialize the node and name it.
    rospy.init_node('robot_imu_publisher')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        asd = RobotImuPublisherNode()
    except rospy.ROSInterruptException:
        pass

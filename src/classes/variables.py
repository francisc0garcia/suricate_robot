import smbus
import struct
import time
import rospy


class RobotVariables(object):
    """
        Get access to robot variables
    """
    def __init__(self):
        """
        init method
        """
         # properties
        self.battery_voltage = 0
        self.motor_1_current = 0
        self.motor_2_current = 0

        # Odometry
        self.velocity_x = 0
        self.velocity_y = 0
        self.velocity_theta = 0

        # IMU info
        self.imu_linear_acceleration_x = 0
        self.imu_linear_acceleration_y = 0
        self.imu_linear_acceleration_z = 0

        self.imu_angular_velocity_x = 0
        self.imu_angular_velocity_y = 0
        self.imu_angular_velocity_z = 0

        self.imu_euler_yaw = 0
        self.imu_euler_roll = 0
        self.imu_euler_pitch = 0

        self.imu_temperature = 0


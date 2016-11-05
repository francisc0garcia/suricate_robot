import rospy
import math

from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler


class ImuPublisher(object):
    degrees2rad = math.pi/180.0

    def __init__(self):
        self.seq = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.yaw_deg = 0

        self.imu_msg = Imu()

        self.imu_msg.orientation_covariance[0] = -1
        self.imu_msg.angular_velocity_covariance[0] = -1
        self.imu_msg.linear_acceleration_covariance[0] = -1

    def set_sequence(self, new_seq):
        self.seq = new_seq

    def publish_info(self, pub_imu, robot_data):
        self.imu_msg.linear_acceleration.x = robot_data.imu_linear_acceleration_x
        self.imu_msg.linear_acceleration.y = robot_data.imu_linear_acceleration_y
        self.imu_msg.linear_acceleration.z = robot_data.imu_linear_acceleration_z

        self.imu_msg.angular_velocity.x = robot_data.imu_angular_velocity_x
        self.imu_msg.angular_velocity.y = robot_data.imu_angular_velocity_y
        self.imu_msg.angular_velocity.z = robot_data.imu_angular_velocity_z

        self.yaw_deg = robot_data.imu_euler_yaw
        self.roll = robot_data.imu_euler_roll
        self.pitch = robot_data.imu_euler_pitch

        '''
        if self.yaw_deg > 180.0:
            self.yaw_deg -= 360.0
        if self.yaw_deg < -180.0:
            self.yaw_deg += 360.0

        if self.roll > 180.0:
            self.roll -= 360.0
        if self.roll < -180.0:
            self.roll += 360.0
        '''

        self.yaw = self.yaw_deg * self.degrees2rad
        self.pitch = self.pitch * self.degrees2rad
        self.roll = self.roll * self.degrees2rad

        # self.yaw = robot_data.imu_euler_yaw
        # self.pitch = robot_data.imu_euler_pitch
        # self.roll = robot_data.imu_euler_roll

        q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
        self.imu_msg.orientation.x = q[0]
        self.imu_msg.orientation.y = q[1]
        self.imu_msg.orientation.z = q[2]
        self.imu_msg.orientation.w = q[3]
        self.imu_msg.header.stamp = rospy.Time.now()
        self.imu_msg.header.frame_id = 'robot_tf'
        self.imu_msg.header.seq = self.seq

        pub_imu.publish(self.imu_msg)

import rospy
import sys
import struct
import time

from classes.variables import *
import serial


class ArduinoSerial(object):
    """
    interface between raspberry and arduino
    """
    def __init__(self):
        """
        init method
        """
        self.ser = serial.Serial('/dev/ttyAMA0', 9600)
        self.output = RobotVariables()
        self.bytes = []

    def get_int(self, data, index):
        self.bytes = data[2*index:(index+1)*2]
        return struct.unpack('d', "".join(map(chr, bytes)))[0]

    def read_block_variables(self):
        try:
            rospy.loginfo("read_block_variables")

            # write_number(1, 0)
            time.sleep(0.003)
            self.ser.write('1')
            self.bytes = self.ser.readline()
            self.output.battery_voltage = self.get_int(self.bytes, 0)
            self.output.motor_1_current = self.get_int(self.bytes, 1)
            self.output.motor_2_current = self.get_int(self.bytes, 2)
            self.output.velocity_x = self.get_int(self.bytes, 3)
            self.output.velocity_y = self.get_int(self.bytes, 4)
            self.output.velocity_theta = self.get_int(self.bytes, 5)
            self.output.imu_linear_acceleration_x = self.get_int(self.bytes, 6)
            self.output.imu_linear_acceleration_y = self.get_int(self.bytes, 7)
            self.output.imu_linear_acceleration_z = self.get_int(self.bytes, 8)
            self.output.imu_angular_velocity_x = self.get_int(self.bytes, 9)
            self.output.imu_angular_velocity_y = self.get_int(self.bytes, 10)
            self.output.imu_angular_velocity_z = self.get_int(self.bytes, 11)
            self.output.imu_euler_yaw = self.get_int(self.bytes, 12)
            self.output.imu_euler_roll = self.get_int(self.bytes, 13)
            self.output.imu_euler_pitch = self.get_int(self.bytes, 14)

        except:
            rospy.loginfo("read_block_variables error: %s ", sys.exc_info()[0])
            pass






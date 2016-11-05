import rospy
import sys
import smbus
import time
import struct
from classes.variables import *


class ArduinoInterface(object):
    """
    interface between raspberry and arduino
    """
    def __init__(self):
        """
        init ArduinoInterface
        """
        self.address = 0x04  # This is the address I2C for Arduino slave
        self.bus = smbus.SMBus(1)
        self.bytes = []
        self.output = RobotVariables()

    def write_number(self, command, value):
        self.bus.write_i2c_block_data(self.address, command, [value])
        return -1

    def get_data(self):
        return self.bus.read_i2c_block_data(self.address, 0)

    def get_float(self, data, index):
        self.bytes = data[4*index:(index+1)*4]
        return struct.unpack('f', "".join(map(chr, bytes)))[0]

    def get_int(self, data, index):
        self.bytes = data[2*index:(index+1)*2]
        return struct.unpack('d', "".join(map(chr, bytes)))[0]

    def read_block_variables(self):
        try:
            # write_number(1, 0)
            time.sleep(0.003)
            self.bytes = self.get_data()
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

    def read_variable(self, id_variable):
        output = 0
        try:
            self.write_number(id_variable, 0)
            time.sleep(0.01)
            data = self.get_data()
            output = self.get_float(data, 0)
        except:
            rospy.loginfo("read_block_variables error: %s ", sys.exc_info()[0])
            pass

        return output

    def update_variable(self, id_variable, value):
        try:
            self.write_number(id_variable, value)
            time.sleep(0.01)
        except:
            rospy.loginfo("update_variable error: %s ", sys.exc_info()[0])
            pass

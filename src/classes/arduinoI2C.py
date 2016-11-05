import sys
import struct
import time
import smbus

from classes.variables import *

class ArduinoI2C(object):
    def __init__(self):
        """
        init method
        """
        self.output = RobotVariables()
        self.bytes = []
        self.bytes_tmp = []
        self.address = 0x50
        self.number_variables = 15
        self.bus = smbus.SMBus(1)

    def get_int(self, index):
        bytes_tmp = list(reversed(self.bytes[2*index:(index+1)*2]))
        return struct.unpack('<H', "".join(map(chr, bytes_tmp)))[0]

    def fill_bytes_arduino(self):
        self.bytes = range(self.number_variables*2)

        for x in range(0, self.number_variables*2):
            add_tmp = self.first_address + x
            self.eeprom_set_current_address(add_tmp)
            self.bytes[x] = bus.read_byte(self.address)
            # self.bytes.append(bus.read_byte(self.address))

    def read_block_variables(self):
        self.fill_bytes_arduino()

        self.output.battery_voltage = self.get_int(0)
        self.output.motor_1_current = self.get_int(1)
        self.output.motor_2_current = self.get_int(2)
        self.output.velocity_x = self.get_int(3)
        self.output.velocity_y = self.get_int(4)
        self.output.velocity_theta = self.get_int(5)
        self.output.imu_linear_acceleration_x = self.get_int(6)
        self.output.imu_linear_acceleration_y = self.get_int(7)
        self.output.imu_linear_acceleration_z = self.get_int(8)
        self.output.imu_angular_velocity_x = self.get_int(9)
        self.output.imu_angular_velocity_y = self.get_int(10)
        self.output.imu_angular_velocity_z = self.get_int(11)
        self.output.imu_euler_yaw = self.get_int(12)
        self.output.imu_euler_roll = self.get_int(13)
        self.output.imu_euler_pitch = self.get_int(14)

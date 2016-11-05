import rospy
import sys
import struct
import time
import smbus
# http://abyz.co.uk/rpi/pigpio/index.html
import pigpio

from classes.variables import *


class ArduinoI2C(object):
    """
    class I2C handler
    """
    def __init__(self):
        """
        init method
        """
        self.output = RobotVariables()
        self.bytes = []
        self.bytes_tmp = []
        self.address = 0x50
        self.first_address = 100
        self.number_variables = 15
        self.pin_working = 27
        self.pin_busy = 22

        self.pi = pigpio.pi()

        self.pi.set_mode(self.pin_working, pigpio.OUTPUT)
        self.pi.set_mode(self.pin_busy, pigpio.INPUT)

    def eeprom_set_current_address(self, addr):
        a1 = addr / 256
        a0 = addr % 256
        bus = smbus.SMBus(1)
        bus.write_i2c_block_data(self.address, a1, [a0])

    def get_int(self, index):
        bytes_tmp = list(reversed(self.bytes[2*index:(index+1)*2]))
        return struct.unpack('<H', "".join(map(chr, bytes_tmp)))[0]

    def fill_bytes_arduino(self):
        self.pi.write(self.pin_working, 1)
        bus = smbus.SMBus(1)

        self.bytes = range(self.number_variables*2)

        for x in range(0, self.number_variables*2):
            add_tmp = self.first_address + x
            self.eeprom_set_current_address(add_tmp)
            self.bytes[x] = bus.read_byte(self.address)
            # self.bytes.append(bus.read_byte(self.address))

        self.pi.write(self.pin_working, 0)

    def read_block_variables(self):
        try:
            busy_flag = self.pi.read(self.pin_busy)

            while busy_flag != 0:
                busy_flag = self.pi.read(self.pin_busy)

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

        except:
            rospy.loginfo("read_block_variables error: %s ", sys.exc_info()[0])
            pass
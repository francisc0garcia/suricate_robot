import rospy
import sys
import struct

import array

# http://abyz.co.uk/rpi/pigpio/index.html
import pigpio
# from periphery import SPI
# from periphery import GPIO

# from quick2wire.gpio import pins, In, Out
from classes.variables import *


class ArduinoSPI(object):
    """
    class SPI handler
    """
    def __init__(self):
        """
        init method
        """
        self.output = RobotVariables()
        self.bytes = []
        self.bytes_tmp = []
        self.first_address = 0x64
        self.number_variables = 15

        # SPI setup
        self.spi_freq = 5000000
        self.pin_slave = 17
        self.pin_working = 27
        self.pin_busy = 22
        self.spi_bus = 0

        # GPIO QuickAccess
        # self.outpin_slave_control = pins.pin(2, Out)  # pin 27 raspberry pi 2

        # GPIO access
        # self.slave_control = GPIO(self.pin_slave, "out")
        # self.slave_control.write(True)

        # FRAM codes:
        self.OPCODE_WREN = 0x06     # Write Enable Latch
        self.OPCODE_WRDI = 0x04     # Reset Write Enable Latch
        self.OPCODE_RDSR = 0x05     # Read Status Register
        self.OPCODE_WRSR = 0x01     # Write Status Register
        self.OPCODE_READ = 0x03     # Read Memory
        self.OPCODE_WRITE = 0x02     # Write Memory

        # self.spi = SPI("/dev/spidev0.0", self.spi_bus, self.spi_freq)
        self.pi = pigpio.pi()
        self.h = self.pi.spi_open(1, self.spi_freq, 3)

        self.pi.set_mode(self.pin_slave, pigpio.OUTPUT)
        self.pi.set_mode(self.pin_working, pigpio.OUTPUT)
        self.pi.set_mode(self.pin_busy, pigpio.INPUT)

        self.pi.write(self.pin_slave, 1)
        self.pi.write(self.pin_slave, 0)

    def get_int(self, index):
        bytes_tmp = self.bytes[2*index:(index+1)*2]
        return struct.unpack('<H', "".join(map(chr, bytes_tmp)))[0]

    def fill_bytes_arduino(self):
        self.pi.write(self.pin_working, 1)

        self.bytes = range(self.number_variables*2)

        for x in range(0, self.number_variables*2):
            add_tmp = self.first_address + x
            part_a = (add_tmp >> 8) & 0xff
            part_b = (add_tmp >> 0) & 0xff

            self.pi.write(self.pin_slave, 0)

            (count, rx_data) = self.pi.spi_xfer(self.h, [self.OPCODE_READ, part_a, part_b, 0x00])
            self.bytes[x] = rx_data[3]
            # self.bytes.append(rx_data[3])
            self.pi.write(self.pin_slave, 1)

        self.pi.write(self.pin_slave, 0)
        self.pi.write(self.pin_working, 0)

    def read_block_variables(self):
        try:
            busy_flag = self.pi.read(self.pin_busy)

            while busy_flag != 0:
                self.pi.write(self.pin_slave, 0)
                busy_flag = self.pi.read(self.pin_busy)

            self.fill_bytes_arduino()

            # self.pi.spi_close(h)
            # rospy.loginfo("self.bytes: %s ", self.bytes)

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

        # finally:
            # self.pi.stop()
            # self.outpin_slave_control.close()

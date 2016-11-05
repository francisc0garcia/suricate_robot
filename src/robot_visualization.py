#!/usr/bin/env python

import rospy
import numpy as np
import math
import smbus
import time

# Adafruit library
from PIL import Image
from PIL import ImageDraw
from Adafruit_LED_Backpack import Matrix8x8


class RobotVisualizer:
    def __init__(self):
        """Init parameters"""
        '''
        List of system variables:
        0. Arduino connection
        1. IMU
        '''
        self.list_system_variables = np.array([0, 0])
        # Create display instance on default I2C address (0x70) and bus number.
        self.display = Matrix8x8.Matrix8x8()
        # Initialize the display. Must be called once before using the display.
        self.display.begin()

        self.demo_matrix()
        time.sleep(0.5)

    def spin(self):
        r = rospy.Rate(100) # 10hz

        while not rospy.is_shutdown():
            self.demo_matrix()
            r.sleep()

    def demo_matrix(self):

        self.display.set_pixel(1, 2, 1)
        self.display.write_display()
        time.sleep(0.5)

        # Draw some shapes using the Python Imaging Library.

        # Clear the display buffer.
        self.display.clear()

        time.sleep(0.5)

        # First create an 8x8 1 bit color image.
        image = Image.new('1', (8, 8))

        # Then create a draw instance.
        draw = ImageDraw.Draw(image)

        # Draw a rectangle with colored outline
        draw.rectangle((0,0,7,7), outline=255, fill=0)

        # Draw an X with two lines.
        draw.line((1,1,6,6), fill=255)
        draw.line((1,6,6,1), fill=255)

        # Draw the image on the display buffer.
        self.display.set_image(image)

        # Draw the buffer to the display hardware.
        self.display.write_display()

        time.sleep(0.5)

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('robot_imu_visualizer')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        object_visualization = RobotVisualizer()
        object_visualization.spin()

    except rospy.ROSInterruptException:
        pass
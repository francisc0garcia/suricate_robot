#!/usr/bin/env python

# Import required Python code.
import rospy
from geometry_msgs.msg import Twist

from src.classes.ArduinoSPI import *
from src.classes.NumericPublisher import *
from src.classes.OdometryPublisher import *
from src.classes.imu_publisher import *
from src.classes.variables import *


class RobotPublisherNode:
    def __init__(self):
        # create object to store robot data
        self.robot_data = RobotVariables()

        # interface with arduino
        # interface_arduino = ArduinoInterface()
        # interface_arduino = ArduinoSerial()
        # interface_arduino = ArduinoI2C()
        interface_arduino = ArduinoSPI()

        # create suscriber for update arduino variables
        rospy.Subscriber("/requested_robot_twist", Twist, self.callback_requested_twist)

        # Create a publisher for imu message
        self.pub_imu = rospy.Publisher('imu_robot', Imu, queue_size=1)
        self.imu_publisher = ImuPublisher()

        # create a publisher for odometry and transform
        self.odometry_pu = rospy.Publisher('odometry_robot', Odometry, queue_size=1)
        self.odometry_publisher = OdometryPublisher()

        # create a numeric publisher for numeric values
        self.motor_1_current_pub = rospy.Publisher('motor_1_current', Float32, queue_size=1)
        self.motor_2_current_pub = rospy.Publisher('motor_2_current', Float32, queue_size=1)
        self.battery_voltage_pub = rospy.Publisher('battery_voltage', Float32, queue_size=1)
        self.numeric_publisher = NumericPublisher()

        self.current_time = rospy.get_time()
        self.last_time = rospy.get_time()

        self.seq = 0
        self.time_temp = 1

        # Odometry
        self.robot_data.velocity_x = 0.1
        self.robot_data.velocity_y = 0.0
        self.robot_data.velocity_theta = 0

        # Main while loop.
        while not rospy.is_shutdown():
            interface_arduino.read_block_variables()

            self.current_time = rospy.get_time()

            # properties
            self.robot_data.battery_voltage = interface_arduino.output.battery_voltage
            self.robot_data.motor_1_current = interface_arduino.output.motor_1_current
            self.robot_data.motor_2_current = interface_arduino.output.motor_2_current

            # IMU info
            self.robot_data.imu_linear_acceleration_x = interface_arduino.output.imu_linear_acceleration_x
            self.robot_data.imu_linear_acceleration_y = interface_arduino.output.imu_linear_acceleration_y
            self.robot_data.imu_linear_acceleration_z = interface_arduino.output.imu_linear_acceleration_z
            self.robot_data.imu_angular_velocity_x = interface_arduino.output.imu_angular_velocity_x
            self.robot_data.imu_angular_velocity_y = interface_arduino.output.imu_angular_velocity_y
            self.robot_data.imu_angular_velocity_z = interface_arduino.output.imu_angular_velocity_z
            self.robot_data.imu_euler_yaw = interface_arduino.output.imu_euler_yaw
            self.robot_data.imu_euler_roll = interface_arduino.output.imu_euler_roll
            self.robot_data.imu_euler_pitch = interface_arduino.output.imu_euler_pitch

            # publish information over ROS
            self.imu_publisher.publish_info(self.pub_imu, self.robot_data)
            self.imu_publisher.set_sequence(self.seq)

            # publish odometry information
            self.odometry_publisher.publish_info(self.odometry_pu, self.robot_data)
            self.odometry_publisher.set_sequence(self.seq)

            # publish numeric information (non standard data)
            self.numeric_publisher.publish_info(self.motor_1_current_pub,
                                                self.motor_2_current_pub,
                                                self.battery_voltage_pub,
                                                self.robot_data)

            self.odometry_publisher.last_time = self.current_time

            self.time_temp += 0.1
            self.seq += 1

    def callback_requested_twist(self, msg):
        rospy.loginfo("Received a /twist message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]" % (msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]" % (msg.angular.x, msg.angular.y, msg.angular.z))

        # TODO: send variables to arduino using I2C

        # for testing purpose
        self.robot_data.velocity_x = msg.linear.x
        self.robot_data.velocity_y = msg.linear.y
        self.robot_data.velocity_theta = msg.angular.x

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('robot_publisher')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = RobotPublisherNode()
    except rospy.ROSInterruptException:
        pass

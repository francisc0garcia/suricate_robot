import rospy
import math

from std_msgs.msg import Float32


class NumericPublisher(object):
    """
    auxiliar class to publish numeric values over ROS
    """
    def __init__(self):
        """
        init method
        """
        self.motor_1_current_msg = Float32()
        self.motor_2_current_msg = Float32()
        self.battery_voltage_msg = Float32()

    def publish_info(self, pub_motor_1, pub_motor_2, pub_bat, robot_data):
        self.motor_1_current_msg.data = robot_data.motor_1_current
        self.motor_2_current_msg.data = robot_data.motor_2_current
        self.battery_voltage_msg.data = robot_data.battery_voltage

        pub_motor_1.publish(self.motor_1_current_msg)
        pub_motor_2.publish(self.motor_2_current_msg)
        pub_bat.publish(self.battery_voltage_msg)

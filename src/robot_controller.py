#!/usr/bin/env python

import rospy
import math
from math import pi, asin, acos
import numpy as np
import time
from threading import Timer,Thread,Event

from classes.variables import *
from classes.imu_publisher import *
from classes.PID import *
from classes.linear_kalman_filer import *
from classes.ControllerPosition import *
from classes.PoseRobot import *

from suricate_robot.cfg import controllerConfig

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from dynamic_reconfigure.server import Server

rad2degrees = 180.0/math.pi

class RobotControllerNode:
    def __init__(self):
        # publish to cmd_vel
        self.pub_vel = rospy.Publisher('/robot/cmd_vel_real', Twist, queue_size=1)

        # Reset output
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        self.pub_vel.publish(self.twist)

        """ Init a new robot controller """
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.offset_roll = 0.0
        self.offset_pitch = 0.0
        self.output = 0.0
        self.desired_x = 0.0
        self.desired_z = 0.0
        self.current_position_x = 0.0
        self.current_vel_x = 0.0

        self.S_kp = 0.0
        self.S_kd = 0.0
        self.S_ki = 0.0
        self.S_windup = 0.0
        self.offset_set_point =  0.03

        self.enable_controller = False

        """Create a position controller"""
        self.controller = ControllerPosition()
        self.distPub = rospy.Publisher('~distance_to_goal', Float32, queue_size=1)
        self.rate = rospy.get_param('~rate', 50.0)
        self.dT = 1 / self.rate
        self.kP = rospy.get_param('~kP', 3.0)
        self.kA = rospy.get_param('~kA', 8.0)
        self.kB = rospy.get_param('~kB', 1.5)
        self.linearTolerance = rospy.get_param('~linear_tolerance', 1)
        self.angularTolerance = rospy.get_param('~angular_tolerance',  5/180*pi)

        self.kP = 0.3
        self.kA = 0.08
        self.kB = 0.015

        self.controller.setConstants(self.kP, self.kA, self.kB)
        self.controller.setLinearTolerance(self.linearTolerance)
        self.controller.setAngularTolerance(self.angularTolerance)
        self.pose = PoseRobot()
        self.initPose()
        self.goal = None

        """Parameters for calibration"""
        self.total_frames_calibration = 40
        self.actual_frame_calibration = 0
        self.offset_position_x = 0
        self.offset_vector = np.zeros((self.total_frames_calibration, 1))
        self.scaling_joy = 1.0

        """Get parameters from launch file"""
        self.S_kp = float(rospy.get_param('~S_kp', 0.0))
        self.S_kd = float(rospy.get_param('~S_kd', 0.0))
        self.S_ki = float(rospy.get_param('~S_ki', 0.0))
        self.S_windup = float(rospy.get_param('~S_windup', 1.0))

        self.scaling_joy = float(rospy.get_param('~scaling_joy', 0))

        self.P_kp = rospy.get_param('P_kp', 0.33)
        self.P_kd = rospy.get_param('P_kd', 0)
        self.P_ki = rospy.get_param('P_ki', 0.05)
        self.P_windup = rospy.get_param('P_windup', 0.8)

        """initialize controllers"""
        self.controller_pid =  PID()
        self.controller_pid.SetPoint = self.offset_set_point  #0.0238
        self.controller_pid.setSampleTime(0.01)
        self.controller_pid.setKp(self.S_kp)
        self.controller_pid.setKd(self.S_kd)
        self.controller_pid.setKi(self.S_ki)
        self.controller_pid.setWindup(self.S_windup)

        self.controller_vel =  PID()
        self.controller_vel.SetPoint = 0.0
        self.controller_vel.setKp(self.P_kp)
        self.controller_vel.setKd(self.P_kd)
        self.controller_vel.setKi(self.P_ki)
        self.controller_vel.setWindup(self.P_windup)

        # kalman filter setup
        self.kalman_joy_A = np.matrix([1.25])
        self.kalman_joy_H = np.matrix([1])
        self.kalman_joy_B = np.matrix([0])
        self.kalman_joy_Q = np.matrix([0.00001])
        self.kalman_joy_R = np.matrix([0.1])
        self.kalman_joy_xhat = np.matrix([0])
        self.kalman_joy_P    = np.matrix([1])
        self.kalman_joy_filter = KalmanFilterLinear(self.kalman_joy_A,self.kalman_joy_B,
                                                    self.kalman_joy_H,self.kalman_joy_xhat,
                                                    self.kalman_joy_P,self.kalman_joy_Q,self.kalman_joy_R)

        rospy.on_shutdown(self.shutdown_node)

        """Setup ROS Publisher/Subscribers"""
        self.sub = rospy.Subscriber('/robot/imu', Imu, self.process_imu_message, queue_size=1)
        self.sub_twist = rospy.Subscriber('/robot/cmd_vel', Twist, self.process_twist_message, queue_size=1)
        self.sub_odometry = rospy.Subscriber('/robot/odom', Odometry, self.process_odometry_message, queue_size=1)
        #self.sub_desired_odometry = rospy.Subscriber('/robot/desired_odom', Odometry, self.process_desired_odometry_message, queue_size=1)

        self.srv = Server(controllerConfig, self.reconfig_callback) # define dynamic_reconfigure callback

        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            self.update_controller()
            rate.sleep()

        #rospy.spin()

    def initPose(self):
        self.pose = PoseRobot()
        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = 0

    def process_imu_message(self, imuMsg):
        quaternion = (
            imuMsg.orientation.x,
            imuMsg.orientation.y,
            imuMsg.orientation.z,
            imuMsg.orientation.w)

        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion)

        self.roll = self.roll * rad2degrees  + self.offset_roll
        self.pitch = self.pitch * rad2degrees + self.offset_pitch
        self.yaw = self.yaw * rad2degrees

        #rospy.loginfo("Angle: roll %f pitch %f yaw %f", self.roll, self.pitch, self.yaw)
        #time.sleep(0.2)

        #test!
        '''
        if (self.roll - 2.25) * imuMsg.orientation.y > 0 :
            self.roll = imuMsg.orientation.y * rad2degrees
        else:
            self.roll = -imuMsg.orientation.y * rad2degrees
        '''
        # self.update_controller()

    def process_odometry_message(self, odometry_msg):
        self.current_position_x = odometry_msg.pose.pose.position.x
        self.current_vel_x = odometry_msg.twist.twist.linear.x

        self.pose = PoseRobot()
        pos = odometry_msg.pose.pose.position
        orientation = odometry_msg.pose.pose.orientation
        self.pose.x = pos.x
        self.pose.y = pos.y
        self.pose.theta = 2 * acos(orientation.w)

    def process_desired_odometry_message(self, odometry_msg):
        self.goal = PoseRobot()
        pos = odometry_msg.pose.pose.position
        orientation = odometry_msg.pose.pose.orientation
        self.goal.x = pos.x
        self.goal.y = pos.y
        self.goal.theta = 2 * asin(orientation.z)

    def process_twist_message(self, twist_msg):
        if twist_msg.linear.x > 0:
            self.enable_controller = True

        self.desired_x = self.kalman_joy_filter.GetCurrentState()[0,0] * self.scaling_joy * 0.9
        self.desired_z = twist_msg.angular.z

        self.kalman_joy_filter.Step(np.matrix([0]), np.matrix([twist_msg.linear.x]))

        self.desired_x = self.bound_limit(self.desired_x, -30.0, 30.0)

        # rospy.loginfo("desired_x: %f measure: %f", self.desired_x, twist_msg.linear.x )
        # rospy.loginfo("changed: desired_x: %f self.desired_z: %f ", self.desired_x, self.desired_z )

    def reconfig_callback(self, config, level):
        self.S_kp = config['S_kp']
        self.S_kd = config['S_kd']
        self.S_ki = config['S_ki']
        self.S_windup = config['S_wu']

        self.P_kp = config['P_kp']
        self.P_kd = config['P_kd']
        self.P_ki = config['P_ki']
        self.P_windup = config['P_wu']

        self.scaling_joy = config['scaling_joy']

        self.controller_pid.setKp(self.S_kp)
        self.controller_pid.setKd(self.S_kd)
        self.controller_pid.setKi(self.S_ki)
        self.controller_pid.setWindup(self.S_windup)
        self.controller_pid.error = 0.0
        self.controller_pid.output = 0.0

        self.controller_vel.setKp(self.P_kp)
        self.controller_vel.setKd(self.P_kd)
        self.controller_vel.setKi(self.P_ki)
        self.controller_vel.setWindup(self.P_windup)
        self.controller_vel.error = 0.0
        self.controller_vel.output = 0.0

        return config

    def update_controller(self):
        self.controller_pid.update(self.pitch)
        self.controller_pid.output = self.bound_limit(self.controller_pid.output , -1, 1)

        self.controller_vel.SetPoint = self.desired_x
        self.controller_vel.update(self.current_vel_x)
        self.controller_vel.output = self.bound_limit(self.controller_vel.output , -1, 1)

        if self.enable_controller:
            self.output = self.controller_pid.output + self.controller_vel.output
            # self.output = self.controller_pid.output + self.desired_x * self.scaling_joy
            self.output = self.bound_limit(self.output , -1, 1)
        else:
            self.output = 0.0
            self.desired_z = 0.0
            self.controller_pid.error = 0.0

        self.twist.linear.x = self.output
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = self.desired_z * self.scaling_joy * 0.35

        self.pub_vel.publish(self.twist)

    def disable_controller(self):
        self.enable_controller = False

    def bound_limit(self, n, minn, maxn):
        return max(min(maxn, n), minn)

    def send_reset_command(self):
        # create a twist message, fill in the details
        self.twist.linear.x = self.output
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = self.desired_z * self.scaling_joy

        self.pub_vel.publish(self.twist)

    def shutdown_node(self):
        rospy.loginfo("Turning off node: robot_controller")
        self.send_reset_command()

# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('robot_imu_controller')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        asd = RobotControllerNode()
    except:

        pass
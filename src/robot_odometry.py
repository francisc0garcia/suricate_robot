#!/usr/bin/env python

"""
    partially taken from:
   diff_tf.py - follows the output of a wheel encoder and
   creates tf and odometry messages.
   some code borrowed from the arbotix diff_controller script
   A good reference: http://rossum.sourceforge.net/papers/DiffSteer/

    Copyright (C) 2012 Jon Stephan.

    diff_controller.py - controller for a differential drive
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.
"""

import rospy
import roslib
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16


from classes.EncoderReader import *

#############################################################################
class DiffTf:
    #############################################################################

    #############################################################################
    def __init__(self):
        #############################################################################
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()

        #### parameters #######
        self.rate = rospy.get_param('~rate', 100.0)  # the rate at which to publish the transform
        self.ticks_meter = float(rospy.get_param('~ticks_meter', 1000))  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(rospy.get_param('~base_width', 0.245)) # The wheel base width in meters

        self.encoder_min = float(rospy.get_param('~encoder_min', -4294964113))
        self.encoder_max = float(rospy.get_param('~encoder_max', 4294964113))
        self.encoder_low_wrap = rospy.get_param('~wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.5 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('~wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.5 + self.encoder_min )

        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()

        self.seq = 0

        # subscriptions

        self.odomPub = rospy.Publisher("/robot/odom", Odometry, queue_size=1)
        self.odomBroadcaster = TransformBroadcaster()

        self.sub_odometry = rospy.Subscriber('/robot/odom_reset', Odometry, self.reset_odometry_message, queue_size=1)

        spi_device = 0
        port_CE0 = 0
        port_CE1 = 1

        self.enc_reader_1 = EncoderReader(spi_device, port_CE0, 1000000)
        self.enc_reader_2 = EncoderReader(spi_device, port_CE1, 1000000)

        self.enc_reader_1.init_encoder()
        self.enc_reader_2.init_encoder()

        # depend on initial value of encoders, to change review first EncoderReader class
        self.offset_encoder = 16777216

    def reset_odometry_message(self, odometry_msg):
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()

        self.enc_reader_1.init_encoder()
        self.enc_reader_2.init_encoder()

        '''
        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = 0
        quaternion.w = 0

        self.odomBroadcaster.sendTransform(
            (0, 0, 0),
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            "robot_tf",
            "map"
        )

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.header.seq = self.seq

        odom.pose.pose.position.x = odometry_msg.pose.pose.position.x
        odom.pose.pose.position.y = odometry_msg.pose.pose.position.y
        odom.pose.pose.position.z = odometry_msg.pose.pose.position.z
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = "robot_tf"
        odom.twist.twist.linear.x = 0
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = 0
        self.odomPub.publish(odom)
        '''

    #############################################################################
    def spin(self):
        #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            #enc_2 = self.right_wheel_callback()
            #enc_1 = self.left_wheel_callback()
            #rospy.loginfo("R: %f, L: %f, enc_1: %f, enc_2: %f" % (self.enc_reader_1.encoder_count, self.enc_reader_2.encoder_count, enc_1, enc_2))

            self.right_wheel_callback()
            self.left_wheel_callback()

            self.update()
            self.set_sequence(self.seq +1)
            r.sleep()

        self.enc_reader_1.close_spi()
        self.enc_reader_2.close_spi()

    #############################################################################
    def update(self):
        #############################################################################
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            # calculate odometry
            if (self.enc_left is None) or (self.enc_right is None) :
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.right - self.enc_right) / self.ticks_meter

            self.enc_left = self.left
            self.enc_right = self.right

            # distance traveled is the average of the two wheels
            d = ( d_left + d_right ) / 2
            # this approximation works (in radians) for small angles
            th = ( d_right - d_left ) / self.base_width
            # calculate velocities
            self.dx = d / elapsed
            self.dr = th / elapsed

            if d != 0:
                # calculate distance traveled in x and y
                x = cos( th ) * d
                y = -sin( th ) * d

                # calculate the final position of the robot
                self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )

            if th != 0:
                self.th = self.th + th

            #rospy.loginfo("x %f y  %f  th  %f enc_left %f enc_right %f ", self.x, self.y, self.th, self.enc_left, self.enc_right)

            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( self.th / 2 )
            quaternion.w = cos( self.th / 2 )

            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                "robot_tf",
                "map"
            )

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.header.seq = self.seq

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = "robot_tf"
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)

            #if abs(self.x) > 500 or abs(self.y) > 500:
            #    self.reset_odometry_message(0)

    def set_sequence(self, new_seq):
        self.seq = new_seq

    def left_wheel_callback(self):
        #############################################################################
        enc = self.offset_encoder + self.enc_reader_1.read_counter()

        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult = self.lmult + 1

        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult = self.lmult - 1

        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min))
        self.prev_lencoder = enc

        return enc

    def right_wheel_callback(self):
        #############################################################################
        enc = - self.offset_encoder - self.enc_reader_2.read_counter()

        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult = self.rmult + 1

        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult = self.rmult - 1

        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

        return enc


if __name__ == '__main__':
    """ main """
    diffTf = DiffTf()
    diffTf.spin()



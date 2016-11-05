import rospy
import math
import numpy as np

from nav_msgs.msg import Odometry
from tf import TransformBroadcaster
from tf import transformations as trans

from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import TransformStamped

class OdometryPublisher(object):
    """
    publish estimate position of robot
    """
    def __init__(self):
        """
        init method
        """
        self.odometry = Odometry()
        self.odometry_broadcaster = TransformBroadcaster()
        self.odometry_transform = TransformStamped()

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0
        self.vy = 0
        self.vth = 0
        self.seq = 0

        self.dt = 0
        self.delta_x = 0
        self.delta_y = 0
        self.delta_th = 0

        self.odometry_quaternion = trans.quaternion_from_euler(0, 0, self.th)

        self.current_time = rospy.get_time()
        self.last_time = rospy.get_time()

    def set_sequence(self, new_seq):
        self.seq = new_seq

    def publish_info(self, pub_odo, robot_data):
        # update velocity information
        self.vx = robot_data.velocity_x
        self.vy = robot_data.velocity_y
        self.vth = robot_data.velocity_theta

        # compute odometry in a typical way given the velocities of the robot
        self.dt = self.current_time - self.last_time
        self.delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * self.dt
        self.delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * self.dt
        self.delta_th = self.vth * self.dt

        self.x += self.delta_x
        self.y += self.delta_y
        self.th += self.delta_th

        self.odometry_quaternion = trans.quaternion_from_euler(0, 0, self.th)

        self.odometry_transform.header.stamp = self.current_time
        self.odometry_transform.header.seq = self.seq
        self.odometry_transform.header.frame_id = "robot_tf"
        self.odometry_transform.child_frame_id = "map"

        self.odometry_transform.transform.translation.x = self.x
        self.odometry_transform.transform.translation.y = self.y
        self.odometry_transform.transform.translation.z = 0.0

        self.odometry_transform.transform.rotation = self.odometry_quaternion

        # send the transform
        self.odometry_broadcaster.sendTransform((self.x, self.y, 0),
                                                self.odometry_transform.transform.rotation,
                                                 rospy.Time.now(),
                                                "robot_tf",
                                                "map")

        # next, we'll publish the odometry message over ROS
        self.odometry.header.stamp = rospy.Time.now()
        self.odometry.header.frame_id = "odom"
        self.odometry.header.seq = self.seq

        self.odometry.pose.pose.position = Point(self.x, self.y, 0)
        self.odometry.pose.pose.orientation = Quaternion(self.odometry_quaternion[0],
                                                         self.odometry_quaternion[1],
                                                         self.odometry_quaternion[2],
                                                         self.odometry_quaternion[3])

        # set the velocity
        self.odometry.child_frame_id = "robot_tf"
        self.odometry.twist.twist.linear.x = self.vx
        self.odometry.twist.twist.linear.y = self.vy
        self.odometry.twist.twist.angular.z = self.vth

        # publish the message
        pub_odo.publish(self.odometry)





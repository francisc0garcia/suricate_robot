#ifndef WRENCH_DRIVER_H
#define WRENCH_DRIVER_H

#include <algorithm>
#include <assert.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/math/gzmath.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Time.hh>
#include <sdf/sdf.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <control_msgs/PidState.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

    class Joint;
    class Entity;

    class WrenchDriver : public ModelPlugin {

    public:
        WrenchDriver();
        ~WrenchDriver();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
        virtual void UpdateChild();
        virtual void FiniChild();

    private:
        GazeboRosPtr gazebo_ros_;
        physics::ModelPtr parent;
        event::ConnectionPtr update_connection_;

        physics::LinkPtr link_left, link_right;
        physics::JointPtr joint_left, joint_right;
        geometry_msgs::Wrench wrench_msg_;

        // ROS STUFF
        ros::Subscriber wrench_subscriber_;
        boost::mutex lock;

        std::string command_topic_, link_left_name, link_right_name, joint_left_name, joint_right_name;
        double max_wheel_torque;

        ros::CallbackQueue queue_;
        boost::thread callback_queue_thread_;

        void QueueThread();

        void cmdWrenchCallback(const geometry_msgs::Wrench::ConstPtr &cmd_msg);
    };
}

#endif //WRENCH_DRIVER_H

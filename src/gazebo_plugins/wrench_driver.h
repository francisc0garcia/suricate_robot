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
        void Reset();

    protected:
        virtual void UpdateChild();
        virtual void resetWorldEvent();
        virtual void FiniChild();

    private:
        GazeboRosPtr gazebo_ros_;
        physics::ModelPtr parent;
        event::ConnectionPtr update_connection_;
        event::ConnectionPtr create_connection_;

        physics::LinkPtr link_wheel;
        physics::JointPtr joint_wheel;

        // ROS STUFF
        ros::Subscriber wrench_subscriber_;
        boost::mutex lock;

        std::string command_topic_;

        ros::CallbackQueue queue_;
        boost::thread callback_queue_thread_;

        void QueueThread();

        void cmdWrenchCallback(const geometry_msgs::Wrench::ConstPtr &cmd_msg);

        bool alive_;

        std::string joint_wheel_name, link_wheel_name;
        double torque_joint_, max_wheel_torque, direction;

        // Update Rate
        double update_rate_, update_period_;

        common::Time last_update_time_;
    };
}

#endif //WRENCH_DRIVER_H

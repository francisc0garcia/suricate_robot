#include "wrench_driver.h"

namespace gazebo {
    // Constructor
    WrenchDriver::WrenchDriver() {
        this->wrench_msg_.force.x = 0;
        this->wrench_msg_.force.y = 0;
        this->wrench_msg_.force.z = 0;
        this->wrench_msg_.torque.x = 0;
        this->wrench_msg_.torque.y = 0;
        this->wrench_msg_.torque.z = 0;
    }

    // Destructor
    WrenchDriver::~WrenchDriver() {
        event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

        // Custom Callback Queue
        this->queue_.clear();
        this->queue_.disable();
        this->callback_queue_thread_.join();
    }

    // Load the controller
    void WrenchDriver::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        this->parent = _parent;

        gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "WrenchDriver"));

        // Make sure the ROS node for Gazebo has already been initialized
        gazebo_ros_->isInitialized();

        // Read parameters from launch file
        gazebo_ros_->getParameter<std::string>(command_topic_, "commandTopic", "cmd_wrench");
        gazebo_ros_->getParameter<std::string>(link_left_name, "link_left", "link_left_name");
        gazebo_ros_->getParameter<std::string>(link_right_name, "link_right", "link_right_name");
        gazebo_ros_->getParameter<std::string>(joint_left_name, "joint_left", "joint_left_name");
        gazebo_ros_->getParameter<std::string>(joint_right_name, "joint_right", "joint_right_name");
        gazebo_ros_->getParameter<double>(max_wheel_torque, "max_wheel_torque", 10.0);

        // Search for links and joints
        link_left = this->parent->GetLink(link_left_name);
        link_right = this->parent->GetLink(link_right_name);
        joint_left = this->parent->GetJoint(joint_left_name);
        joint_right = this->parent->GetJoint(joint_right_name);

        if (!joint_left || !joint_right || !link_left || !link_right) {
            ROS_ERROR("Not found: (!joint_left || !joint_right || !link_left || !link_right)");
            return;
        }

        // ROS: Subscribe to wrench command topic (usually "cmd_wrench")
        ROS_INFO("%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str());

        ros::SubscribeOptions so =
                ros::SubscribeOptions::create<geometry_msgs::Wrench>(command_topic_, 1,
                                                                    boost::bind(&WrenchDriver::cmdWrenchCallback,
                                                                                this, _1),
                                                                    ros::VoidPtr(), &queue_);

        wrench_subscriber_ = gazebo_ros_->node()->subscribe(so);
        ROS_INFO("%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str());

        // start custom queue for driver
        this->callback_queue_thread_ =
                boost::thread(boost::bind(&WrenchDriver::QueueThread, this));

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_ =
                event::Events::ConnectWorldUpdateBegin(boost::bind(&WrenchDriver::UpdateChild, this));

    }

    // Update force and torque
    void WrenchDriver::UpdateChild() {
        this->lock.lock();

        //Apply force/torque to links
        //this->joint_left->SetForce(1, this->wrench_msg_.torque.y);
        //this->joint_right->SetForce(1, this->wrench_msg_.torque.y);

        // Robot specific
        double radius = 0.16;
        double L_p = 0.6655;

        double dx = this->wrench_msg_.force.x; // vel->linear.x;
        double dr = this->wrench_msg_.torque.z; // vel->angular.z;

        double right = ((100/15)/(2*radius)) * (dx + L_p * dr);
        double left = ((100/15)/(2*radius)) * (dx - L_p * dr);

        this->joint_left->SetForce(1, right);
        this->joint_right->SetForce(1, left);

        this->lock.unlock();
    }

    // Finalize the controller
    void WrenchDriver::FiniChild() {
        queue_.clear();
        queue_.disable();
        gazebo_ros_->node()->shutdown();
        callback_queue_thread_.join();
    }

    void WrenchDriver::cmdWrenchCallback(const geometry_msgs::Wrench::ConstPtr &cmd_msg) {
        this->wrench_msg_.force.x = cmd_msg->force.x;
        this->wrench_msg_.force.y = cmd_msg->force.y;
        this->wrench_msg_.force.z = cmd_msg->force.z;
        this->wrench_msg_.torque.x = cmd_msg->torque.x;
        this->wrench_msg_.torque.y = cmd_msg->torque.y;
        this->wrench_msg_.torque.z = cmd_msg->torque.z;
    }

    void WrenchDriver::QueueThread() {
        static const double timeout = 0.01;

        while (gazebo_ros_->node()->ok()) {
            queue_.callAvailable(ros::WallDuration(timeout));
        }

        this->FiniChild();
    }

    GZ_REGISTER_MODEL_PLUGIN (WrenchDriver)
}


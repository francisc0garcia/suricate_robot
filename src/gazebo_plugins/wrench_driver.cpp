#include "wrench_driver.h"

namespace gazebo {
    WrenchDriver::WrenchDriver() {}

    // Destructor
    WrenchDriver::~WrenchDriver() {}

    // Load the controller
    void WrenchDriver::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        this->parent = _parent;

        gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "WrenchDriver"));

        // Make sure the ROS node for Gazebo has already been initialized
        gazebo_ros_->isInitialized();

        gazebo_ros_->getParameter<std::string>(command_topic_, "commandTopic", "cmd_wrench");
        gazebo_ros_->getParameter<std::string>(joint_wheel_name, "joint_wheel", "joint_wheel_name");
        gazebo_ros_->getParameter<std::string>(link_wheel_name, "link_wheel", "link_wheel_name");
        gazebo_ros_->getParameter<double>(max_wheel_torque, "max_wheel_torque", 5.0);
        gazebo_ros_->getParameter<double>(update_rate_, "updateRate", 100.0);
        gazebo_ros_->getParameter<double>(direction, "direction", 1);

        link_wheel = this->parent->GetLink(link_wheel_name);
        joint_wheel = gazebo_ros_->getJoint(parent, "joint_wheel", "_joint_wheel");
        joint_wheel->SetParam("fmax", 0, max_wheel_torque);

        if (!joint_wheel) {
            ROS_ERROR("Not found: (joint_wheel)");
            return;
        }

        // Initialize update rate stuff
        if (this->update_rate_ > 0.0)
            this->update_period_ = 1.0 / this->update_rate_;
        else
            this->update_period_ = 0.0;
        last_update_time_ = parent->GetWorld()->GetSimTime();

        alive_ = true;

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

        this->create_connection_ =
                event::Events::ConnectWorldReset(boost::bind(&WrenchDriver::resetWorldEvent, this));
    }

    void WrenchDriver::Reset() {
        last_update_time_ = parent->GetWorld()->GetSimTime();
        joint_wheel->SetParam("fmax", 0, max_wheel_torque);
    }

    // Update the controller
    void WrenchDriver::UpdateChild() {
        common::Time current_time = parent->GetWorld()->GetSimTime();
        double seconds_since_last_update = (current_time - last_update_time_).Double();
        //joint_wheel
        if (seconds_since_last_update > update_period_) {

            // TODO: does not work!

            // Change force:
            //joint_wheel->SetForce(0, torque_joint_);
            //joint_wheel->SetVelocity(0, 5);
            //this->parent->GetJoint(joint_wheel_name)->SetForce(0, 100);

            //link_wheel->SetTorque(math::Vector3(0, direction * torque_joint_, 0));
            //link_wheel->SetTorque(math::Vector3(20, 0, 0));

            joint_wheel->SetParam( "vel", 0, 10 );

            //ROS_INFO("Set torque_joint_ to: %f" , torque_joint_);
            last_update_time_ += common::Time(update_period_);
        }
    }

    void WrenchDriver::resetWorldEvent() {
        ROS_INFO("WrenchDriver::resetWorldEvent()");
    }

    // Finalize the controller
    void WrenchDriver::FiniChild() {
        alive_ = false;
        queue_.clear();
        queue_.disable();
        gazebo_ros_->node()->shutdown();
        callback_queue_thread_.join();
    }

    void WrenchDriver::cmdWrenchCallback(const geometry_msgs::Wrench::ConstPtr &cmd_msg) {
        boost::mutex::scoped_lock scoped_lock(lock);
        torque_joint_ = cmd_msg->force.x;
    }

    void WrenchDriver::QueueThread() {
        static const double timeout = 0.01;

        while (alive_ && gazebo_ros_->node()->ok()) {
            queue_.callAvailable(ros::WallDuration(timeout));
        }

        this->Reset();
        this->FiniChild();
    }

    GZ_REGISTER_MODEL_PLUGIN (WrenchDriver)
}


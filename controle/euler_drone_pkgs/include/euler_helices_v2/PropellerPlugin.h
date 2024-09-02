#ifndef PROPELLER_PLUGIN_HH
#define PROPELLER_PLUGIN_HH

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>

// C++ Standard Library
#include <thread>

namespace gazebo
{
    class PropellerPlugin : public ModelPlugin
    {
    public:
        PropellerPlugin();
        ~PropellerPlugin();

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    private:
        void OnUpdate();
        void CalculateForces();

        // ROS NodeHandle and Publisher
        ros::NodeHandle nh_;
        ros::Publisher propeller_force_publisher_;

        // Gazebo connections and pointers
        event::ConnectionPtr update_connection_;
        physics::ModelPtr model;

        // Parameters
        std::string frame_id_;
        std::string propeller_force_topic_;
        double update_period_;
        bool publish_force_;

        // Timing
        common::Time last_update_time_;
    };
}

#endif

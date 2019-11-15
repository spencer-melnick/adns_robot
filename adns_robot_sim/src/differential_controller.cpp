#include "adns_robot_sim/differential_controller.hpp"

#include <ros/subscribe_options.h>

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(DifferentialControllerPlugin);

    void DifferentialControllerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM_NAMED("differential_controller", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        for (auto& tag: DifferentialControllerPlugin::REQUIRED_TAGS)
        {
            if (!_sdf->HasElement(tag))
            {
                ROS_FATAL_NAMED("differential_controller", "Tag %s was not found, but is required for plugin differential_controller", tag.c_str());
                return;
            }
        }

        std::string robot_namespace = _sdf->GetElement("namespace")->GetValue()->GetAsString();
        node_.reset(new ros::NodeHandle(robot_namespace + "/differential_controller"));
        ROS_INFO_NAMED("differential_controller", "Spawned differential_controller node in namespace %s", robot_namespace.c_str());

        std::string differential_velocity_topic = _sdf->GetElement("topic")->GetValue()->GetAsString();
        ros::SubscribeOptions subscribe_options = ros::SubscribeOptions::create<adns_robot_core::differential_velocity>(
            differential_velocity_topic,
            1,
            boost::bind(&DifferentialControllerPlugin::OnVelocityMessage, this, _1),
            ros::VoidPtr(),
            &callback_queue_);

        subscriber_ = node_->subscribe(subscribe_options);
        ROS_INFO_NAMED("differential_controller", "Subscribed to control velocity topic %s", differential_velocity_topic.c_str());

        queue_thread_ = std::thread(std::bind(&DifferentialControllerPlugin::QueueThread, this));
    }

    void DifferentialControllerPlugin::OnVelocityMessage(const adns_robot_core::differential_velocityConstPtr& message)
    {
        // do something!
        ROS_INFO_STREAM_NAMED("differential_controller", "Velocity updated");
    }

    void DifferentialControllerPlugin::QueueThread()
    {
        ROS_INFO_NAMED("differential_controller", "Spinning up control velocity thread");

        while (node_->ok())
        {
            callback_queue_.callAvailable(ros::WallDuration(DifferentialControllerPlugin::TIMEOUT));
        }
    }
}
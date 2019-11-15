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

        AssignDriveJoints(_parent, _sdf, "left_drive_joint", left_drive_joints_);
        AssignDriveJoints(_parent, _sdf, "right_drive_joint", right_drive_joints_);

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
        for (auto& joint: left_drive_joints_)
        {
            joint->SetParam("vel", 0, message->left_velocity);
        }

        for (auto& joint: right_drive_joints_)
        {
            joint->SetParam("vel", 0, message->right_velocity);
        }
    }

    void DifferentialControllerPlugin::QueueThread()
    {
        ROS_INFO_NAMED("differential_controller", "Spinning up control velocity thread");

        while (node_->ok())
        {
            callback_queue_.callAvailable(ros::WallDuration(DifferentialControllerPlugin::TIMEOUT));
        }
    }

    void DifferentialControllerPlugin::AssignDriveJoints(physics::ModelPtr model, sdf::ElementPtr sdf, std::string tag_name, std::vector<physics::JointPtr>& array)
    {
        sdf::ElementPtr drive_joint_element = sdf->GetElement(tag_name);
        physics::JointPtr drive_joint;

        while (drive_joint_element)
        {
            drive_joint = model->GetJoint(drive_joint_element->GetValue()->GetAsString());

            if (drive_joint)
            {
                drive_joint->SetParam("fmax", 0, 100.0);
                array.push_back(drive_joint);
            }

            drive_joint_element = drive_joint_element->GetNextElement(tag_name);
        }

        ROS_INFO_NAMED("differential_controller", "%d joints assigned to %s", static_cast<int>(array.size()), tag_name.c_str());
    }
}
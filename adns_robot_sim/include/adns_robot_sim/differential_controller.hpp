#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>

#include <thread>
#include <vector>

#include "adns_robot_core/differential_velocity.h"

namespace gazebo
{
    class DifferentialControllerPlugin: public ModelPlugin
    {
        public:
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

        protected:
            void OnVelocityMessage(const adns_robot_core::differential_velocityConstPtr& message);
            void QueueThread();
            void AssignDriveJoints(physics::ModelPtr model, sdf::ElementPtr sdf, std::string tag_name, std::vector<physics::JointPtr>& array);

            const double TIMEOUT = 0.01;
            const double MAX_FORCE_DEFAULT = 1.0;
            const std::vector<std::string> REQUIRED_TAGS = {"namespace", "topic"};

        private:
            std::unique_ptr<ros::NodeHandle> node_;
            ros::Subscriber subscriber_;
            ros::CallbackQueue callback_queue_;
            std::thread queue_thread_;

            std::vector<physics::JointPtr> left_drive_joints_;
            std::vector<physics::JointPtr> right_drive_joints_;
    };
}
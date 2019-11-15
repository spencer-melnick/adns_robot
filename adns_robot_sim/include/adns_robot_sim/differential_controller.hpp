#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <gazebo/gazebo.hh>

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

            const double TIMEOUT = 0.01;
            const std::vector<std::string> REQUIRED_TAGS = {"namespace", "topic"};

        private:
            std::unique_ptr<ros::NodeHandle> node_;
            ros::Subscriber subscriber_;
            ros::CallbackQueue callback_queue_;
            std::thread queue_thread_;
    };
}
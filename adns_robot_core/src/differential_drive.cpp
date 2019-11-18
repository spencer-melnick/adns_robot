#include <ros/ros.h>
#include <ros/console.h>

#include <adns_robot_core/differential_velocity.h>
#include <geometry_msgs/Twist.h>

#include <functional>

class DifferentialDriveNode
{
    public:
        DifferentialDriveNode(ros::NodeHandle& node)
        {
            publisher_ = node.advertise<adns_robot_core::differential_velocity>("drive_velocity", 1);

            ROS_WARN_COND_NAMED(!node.getParam("wheel_base", wheel_base), "differential_drive", "Wheel base not specified in parameter server");
            ROS_WARN_COND_NAMED(!node.getParam("wheel_radius", wheel_radius), "differential_drive", "Wheel radius not specified in parameter server");

            subscriber_ = node.subscribe("cmd_vel", 1, &DifferentialDriveNode::OnCommandVelocityUpdate, this);
        }

        void OnCommandVelocityUpdate(const geometry_msgs::TwistConstPtr& msg)
        {
            adns_robot_core::differential_velocity velocity;

            double forward_velocity = msg->linear.x;
            double angular_velocity = msg->angular.z;

            velocity.left_velocity = forward_velocity / wheel_radius - angular_velocity * wheel_base / (2 * wheel_radius);
            velocity.right_velocity = forward_velocity / wheel_radius + angular_velocity * wheel_base / (2 * wheel_radius);

            publisher_.publish(velocity);
        }

        double wheel_base = 1.0;
        double wheel_radius = 1.0;

    private:
        ros::Publisher publisher_;
        ros::Subscriber subscriber_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "differential_drive");
    ros::NodeHandle node;

    DifferentialDriveNode drive(node);

    ros::spin();

    return 0;
}

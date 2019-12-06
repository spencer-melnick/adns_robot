#include <ros/ros.h>

#include <ros/publisher.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <nav_msgs/Odometry.h>

const char node_name[] = "pose_tf_broadcaster";

class PoseBroadcasterNode
{
    public:
        PoseBroadcasterNode(ros::NodeHandle& node)
        {
            std::string input_topic = "fiducial_transforms_unrotated";
            std::string output_topic = "/fiducial_transforms";
            node.getParam("input_topic", input_topic);
            node.getParam("camera_link_name", camera_link_name_);
            node.getParam("output_topic", output_topic);

            subscriber_ = node.subscribe(input_topic, 1, &PoseBroadcasterNode::OnFiducialTransformsUpdates, this);
            publisher_ = node.advertise<fiducial_msgs::FiducialTransformArray>(output_topic, 5);
        }

    protected:
        void OnFiducialTransformsUpdates(fiducial_msgs::FiducialTransformArray message)
        {
            for (auto& transform : message.transforms)
            {
                tf2::Transform input_transform;
                tf2::fromMsg(transform.transform, input_transform);

                // The marker transform is in the wrong coordinate system, rotate it to fix it
                tf2::Matrix3x3 coordinate_correction_matrix;
                coordinate_correction_matrix.setRPY(-M_PI_2, 0, -M_PI_2);
                tf2::Transform coordinate_correction (coordinate_correction_matrix);
                tf2::Transform output_transform = coordinate_correction * input_transform;  

                transform.transform = tf2::toMsg(output_transform);
            }

            publisher_.publish(message);
        }

    private:
        std::string camera_link_name_ = "camera_link";
        std::string base_link_name_ = "base_link";

        ros::Subscriber subscriber_;
        ros::Publisher publisher_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle node("~");

    PoseBroadcasterNode broadcaster_node(node);

    ros::spin();

    return 0;
}

#include <ros/ros.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <fiducial_msgs/FiducialTransformArray.h>

const char node_name[] = "pose_tf_broadcaster";

class PoseBroadcasterNode
{
    public:
        PoseBroadcasterNode(ros::NodeHandle& node)
        {
            std::string transform_topic = "fiducial_transforms";
            node.getParam("transform_topic", transform_topic);
            node.getParam("camera_link_name", camera_link_name_);

            subscriber_ = node.subscribe(transform_topic, 1, &PoseBroadcasterNode::OnFiducialTransformsUpdates, this);
        }

    protected:
        void OnFiducialTransformsUpdates(const fiducial_msgs::FiducialTransformArrayConstPtr& message)
        {
            for (auto transform : message->transforms)
            {
                int marker_id = transform.fiducial_id;

                tf2::Transform marker_transform;
                tf2::fromMsg(transform.transform, marker_transform);
                tf2::Quaternion coordinate_correction;
                coordinate_correction.setRPY(M_PI_2, 0, -M_PI_2);
                marker_transform = marker_transform.inverse();
                marker_transform.setRotation(coordinate_correction * marker_transform.getRotation());
                
                geometry_msgs::TransformStamped transform_message;
                transform_message.child_frame_id = "camera_link";
                transform_message.transform = tf2::toMsg(marker_transform);
                transform_message.header.frame_id = "aruco_" + std::to_string(marker_id);
                transform_message.header.stamp = message->header.stamp;

                broadcaster_.sendTransform(transform_message);
            }
        }

    private:
        ros::Subscriber subscriber_;
        tf2_ros::TransformBroadcaster broadcaster_;
        std::string camera_link_name_ = "camera_link";
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle node("~");

    PoseBroadcasterNode broadcaster_node(node);

    ros::spin();

    return 0;
}

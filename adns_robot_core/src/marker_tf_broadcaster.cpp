#include <ros/ros.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <vector>
#include <string>

const char node_name[] = "marker_tf_broadcaster";

int main(int argc, char** argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle node;

    tf2_ros::StaticTransformBroadcaster broadcaster;
    std::vector<geometry_msgs::TransformStamped> messages;

    XmlRpc::XmlRpcValue markers_list;
    ROS_WARN_COND_NAMED(!node.getParam("/aruco_markers", markers_list), node_name, "No ArUco markers specified");

    for (int i = 0; i < markers_list.size(); i++)
    {
        XmlRpc::XmlRpcValue marker_xml = markers_list[i];
        XmlRpc::XmlRpcValue marker_pose = marker_xml["pose"];
        int marker_id = marker_xml["id"];

        geometry_msgs::TransformStamped message;

        message.transform.translation.x = marker_pose[0];
        message.transform.translation.y = marker_pose[1];
        message.transform.translation.z = marker_pose[2];

        tf2::Quaternion rotation;
        rotation.setRPY(marker_pose[3], marker_pose[4], marker_pose[5]);

        message.transform.rotation.x = rotation.getX();
        message.transform.rotation.y = rotation.getY();
        message.transform.rotation.z = rotation.getZ();
        message.transform.rotation.w = rotation.getW();

        message.header.stamp = ros::Time::now();
        message.header.frame_id = "world";
        message.child_frame_id = "aruco_" + std::to_string(marker_id);

        messages.push_back(message);
    }

    broadcaster.sendTransform(messages);
    ROS_INFO_NAMED(node_name, "%d marker transforms published", static_cast<int>(messages.size()));
    ros::spin();

    return 0;
}

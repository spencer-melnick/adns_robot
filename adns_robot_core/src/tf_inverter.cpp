#include <ros/ros.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <xmlrpcpp/XmlRpc.h>

#include <vector>
#include <utility>
#include <string>

const char node_name[] = "tf_inverter";

int main(int argc, char** argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle node;

    std::vector<std::pair<std::string, std::string>> transform_pairs;

    double rate_hz = 20.0;
    node.getParam("publish_rate", rate_hz);
    ros::Rate rate(rate_hz);

    XmlRpc::XmlRpcValue inverted_transforms;
    ROS_WARN_COND_NAMED(!node.getParam("/inverted_transforms", inverted_transforms), node_name, "No inverted_transforms specified");

    for (int i = 0; i < inverted_transforms.size(); i++)
    {
        XmlRpc::XmlRpcValue transform_xml = inverted_transforms[i];
        transform_pairs.push_back({transform_xml["parent"], transform_xml["child"]});
    }

    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener listener(tf2_buffer);
    tf2_ros::TransformBroadcaster broadcaster;

    while (node.ok())
    {
        for (auto i : transform_pairs)
        {
            try
            {
                geometry_msgs::TransformStamped transform_message = tf2_buffer.lookupTransform(i.first, i.second, ros::Time(0.0));

                tf2::Transform transform; 
                tf2::fromMsg(transform_message.transform, transform);
                std::string previous_child = transform_message.child_frame_id;
                std::string previous_parent = transform_message.header.frame_id;

                transform_message.transform = tf2::toMsg(transform.inverse());
                transform_message.child_frame_id = previous_parent;
                transform_message.header.frame_id = previous_child;

                broadcaster.sendTransform(transform_message);
            }
            catch(tf2::TransformException& e)
            {

            }
        }
    }

    return 0;
}

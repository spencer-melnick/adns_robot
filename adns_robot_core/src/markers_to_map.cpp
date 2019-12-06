#include <ros/ros.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <vector>
#include <string>
#include <fstream>

const char node_name[] = "markers_to_map";

int main(int argc, char** argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle node("~");

    XmlRpc::XmlRpcValue markers_list;
    std::string map_file_path = "map.txt";

    ROS_WARN_COND_NAMED(!node.getParam("/aruco_markers", markers_list), node_name, "No ArUco markers specified");
    ROS_WARN_COND_NAMED(!node.getParam("map_path", map_file_path), node_name, "No map path specified, using default");


    // Sensor values
    double starting_variance = 0.01;
    int starting_observations = 10;

    node.getParam("starting_variance", starting_variance);
    node.getParam("starting_observations", starting_observations);

    std::ofstream map_file(map_file_path, std::ios::trunc);

    if (!map_file.is_open())
    {
        ROS_FATAL_NAMED(node_name, "Could not open map file: %s", map_file_path.c_str());
        return 1;
    }

    ROS_INFO_NAMED(node_name, "%d markers being added to map file", markers_list.size());

    for (int i = 0; i < markers_list.size(); i++)
    {
        XmlRpc::XmlRpcValue marker_xml = markers_list[i];
        XmlRpc::XmlRpcValue marker_pose = marker_xml["pose"];
        int marker_id = marker_xml["id"];

        map_file << std::to_string(marker_id) << " ";

        for (int j = 0; i < 6; j++)
        {
            map_file << std::to_string(static_cast<double>(marker_pose[j])) << " ";
        }

        map_file << std::to_string(starting_variance) << " " << std::to_string(starting_observations) << " \n";
    }


    map_file.close();
    ROS_INFO_NAMED(node_name, "Map file %s created", map_file_path.c_str());
    return 0;
}

#include "aruco_markers_gazebo/model_generator.h"

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include <xmlrpcpp/XmlRpc.h>
#include <tf2/LinearMath/Quaternion.h>

// OpenCV includes
#include <opencv/cv.h>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>


// Third-party system libraries
#include <boost/filesystem.hpp>

// Gazebo includes
#include <gazebo_msgs/SpawnModel.h>

// Standard libraries
#include <fstream>
#include <regex>
#include <stdlib.h>



// Constants

const char node_name[] = "model_generator";

std::unordered_map<std::string, int> dictionary_strings = {
    {"4x4_250", cv::aruco::DICT_4X4_250},
    {"4x4_1000", cv::aruco::DICT_4X4_1000},
    {"5x5_250", cv::aruco::DICT_5X5_250},
    {"5x5_1000", cv::aruco::DICT_5X5_1000},
    {"6x6_250", cv::aruco::DICT_6X6_250},
    {"6x6_1000", cv::aruco::DICT_6X6_1000},
    {"7x7_250", cv::aruco::DICT_7X7_250},
    {"7x7_1000", cv::aruco::DICT_7X7_1000}
};




// Utility functions

int lookup_dictionary_id_by_name(const std::string& name)
{
    int dictionary_id = cv::aruco::DICT_5X5_250;
    auto result = dictionary_strings.find(name);

    if (result != dictionary_strings.end())
    {
        dictionary_id = result->second;
    }
    else
    {
        ROS_WARN_NAMED(node_name, "%s is not a valid dictionary - using default", name.c_str());
    }
    

    return dictionary_id;
}

std::string lookup_dictionary_name_by_id(int id)
{
    for (auto i : dictionary_strings)
    {
        if (i.second == id)
        {
            return i.first;
        }
    }

    ROS_WARN_NAMED(node_name, "Dictionary %d does not have a name associated with it", id);
    return "";
}

std::string load_file_to_string(const std::string& filename)
{
    std::string file_content;

    std::ifstream file(filename);

    if (file.is_open())
    {
        file.seekg(0, std::ios::end);
        file_content.resize(file.tellg());
        file.seekg(0, std::ios::beg);

        file.read(&file_content[0], file_content.size());
        file.close();
    }
    else
    {
        ROS_WARN_NAMED(node_name, "Cannot open file %s", filename.c_str());
    }

    return file_content;
}





// Core functions

int main(int argc, char** argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    // Load parameters
    std::string dictionary_string;
    ROS_WARN_COND_NAMED(!node.getParam("/aruco_dictionary", dictionary_string), node_name, "ArUco dictionary not specified");
    int dictionary_id = lookup_dictionary_id_by_name(dictionary_string);

    XmlRpc::XmlRpcValue markers;
    ROS_WARN_COND_NAMED(!node.getParam("/aruco_markers", markers), node_name, "No ArUco markers specified");

    bool should_spawn_gazebo_models = false;
    ROS_WARN_COND_NAMED(!private_node.getParam("should_spawn_gazebo_models", should_spawn_gazebo_models), node_name, "No parameter specified, not spawning models");
    ros::ServiceClient service_client;

    if (should_spawn_gazebo_models)
    {
        ros::service::waitForService("gazebo/spawn_sdf_model");
        service_client = node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
        ROS_WARN_COND_NAMED(!service_client, node_name, "Unable to create service client");
    }

    // Define output paths
    std::string default_directory = std::string(getenv("HOME")) + "/.ros/gazebo_models/aruco";
    std::string output_directory = private_node.param<std::string>("model_output_path", default_directory);
    std::string package_path = ros::package::getPath("aruco_markers_gazebo");

    // Load template files into memory
    std::string model_template = load_file_to_string(package_path + "/models/marker_template.sdf");
    std::string material_template = load_file_to_string(package_path + "/materials/scripts/template.material");
    std::string config_template = load_file_to_string(package_path + "/models/marker_template.config");

    for (int i = 0; i < markers.size(); i++)
    {
        // Populate struct from XML value
        // TODO: add error checking
        XmlRpc::XmlRpcValue marker_xml = markers[i];
        XmlRpc::XmlRpcValue pose = marker_xml["pose"];
        MarkerInfo marker;
        marker.id = marker_xml["id"];
        marker.side_length = marker_xml["side_length"];

        // Populate position
        marker.pose.position.x = pose[0];
        marker.pose.position.y = pose[1];
        marker.pose.position.z = pose[2];

        // Get quaternion from Euler angles
        tf2::Quaternion orientation;
        orientation.setRPY(pose[3], pose[4], pose[5]);

        // Populate orientation from quaternion
        marker.pose.orientation.x = orientation.getX();
        marker.pose.orientation.y = orientation.getY();
        marker.pose.orientation.z = orientation.getZ();
        marker.pose.orientation.w = orientation.getW();

        std::string marker_name = dictionary_string + "_" + std::to_string(marker.id);

        // Create output directories
        std::string model_directory = output_directory + "/" + marker_name;
        std::string mesh_directory = model_directory + "/meshes";
        boost::filesystem::create_directories(mesh_directory);
        boost::filesystem::create_directories(model_directory);

        // Copy flat plane mesh to model directory
        boost::filesystem::copy_file(package_path + "/meshes/unit_plane.dae", mesh_directory + "/unit_plane.dae", boost::filesystem::copy_option::overwrite_if_exists);

        // Fill in templates
        std::string rendered_model = std::regex_replace(model_template, std::regex("\\[marker_name\\]"), marker_name);
        rendered_model = std::regex_replace(rendered_model, std::regex("\\[side_length\\]"), std::to_string(marker.side_length));
        std::string rendered_config = std::regex_replace(config_template, std::regex("\\[marker_name\\]"), marker_name);

        // Output files
        std::ofstream model_file(model_directory + "/model.sdf");
        std::ofstream config_file(model_directory + "/model.config");
        model_file << rendered_model;
        config_file << rendered_config;

        // Close all files
        model_file.close();
        config_file.close();

        // Render images
        generate_aruco_image(dictionary_id, marker.id, mesh_directory);

        if (should_spawn_gazebo_models)
        {
            gazebo_msgs::SpawnModel spawn_model;
            spawn_model.request.initial_pose = marker.pose;
            spawn_model.request.model_xml = rendered_model;
            spawn_model.request.model_name = marker_name;
            spawn_model.request.robot_namespace = "aruco_markers";

            ROS_WARN_COND_NAMED(!service_client.call(spawn_model), node_name, "Unable to call spawn_sdf_model service");
            ROS_WARN_COND_NAMED(!spawn_model.response.success, node_name, "Unable to spawn ArUco model %s: %s", spawn_model.request.model_xml.c_str(),
                spawn_model.response.status_message.c_str());
        }
    }

    return 0;
}

bool generate_aruco_image(int dictionary_id, int marker_id, const std::string& directory)
{
    cv::Ptr<cv::aruco::Dictionary> dictionary_pointer = cv::aruco::getPredefinedDictionary(dictionary_id);
    std::string dictionary_name = lookup_dictionary_name_by_id(dictionary_id);

    std::string file_name = "marker.png";
    std::string file_path = directory + "/" + file_name;

    cv::Mat marker_image;
    cv::aruco::drawMarker(dictionary_pointer, marker_id, 1024, marker_image);

    boost::filesystem::create_directories(directory);
    if (!cv::imwrite(file_path, marker_image))
    {
        ROS_WARN_NAMED(node_name, "Could not write marker image: %s", file_path.c_str());
        return false;
    }

    ROS_INFO_NAMED(node_name, "Writing ArUco image to %s", file_path.c_str());
    return true;
}


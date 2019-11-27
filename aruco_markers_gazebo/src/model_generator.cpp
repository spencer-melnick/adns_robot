#include "aruco_markers_gazebo/model_generator.h"

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>

// OpenCV includes
#include <opencv/cv.h>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>


// Third-party system libraries
#include <boost/filesystem.hpp>
#include <fmt/format.h>
#include <nlohmann/json.hpp>

// Standard libraries
#include <fstream>
#include <regex>
#include <stdlib.h>

using nlohmann::json;



// Constants

const char node_name[] = "model_generator";

std::unordered_map<std::string, int> dictionary_strings = {
    {"4x4", cv::aruco::DICT_4X4_250},
    {"5x5", cv::aruco::DICT_5X5_250},
    {"6x6", cv::aruco::DICT_6X6_250},
    {"7x7", cv::aruco::DICT_7X7_250}
};




// Utility functions

void to_json(json& j, const MarkerInfo& marker)
{
    j = json{{"id", marker.id}, {"side_length", marker.side_length}};
}

void from_json(const json& j, MarkerInfo& marker)
{
    j.at("id").get_to(marker.id);
    j.at("side_length").get_to(marker.side_length);
}

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
    ros::NodeHandle node("~");
    std::string input_path;

    if (!node.getParam("input_file", input_path))
    {
        ROS_FATAL_NAMED(node_name, "No input file specified");
        return 1;
    }

    std::ifstream input_stream(input_path);

    if (!input_stream.is_open())
    {
        ROS_FATAL_NAMED(node_name, "Cannot open input file %s", input_path.c_str());
        return 1;
    }

    json input;
    input_stream >> input;
    input_stream.close();

    ROS_WARN_COND_NAMED(!input.contains("dictionary"), node_name, "Input file %s does not specify a dictionary type - using default", input_path.c_str());
    std::string dictionary_string = input.value<std::string>("dictionary", "6x6");
    int dictionary_id = lookup_dictionary_id_by_name(dictionary_string);

    std::string default_directory = std::string(getenv("HOME")) + "/.ros/gazebo_models/aruco";
    std::string output_directory = node.param<std::string>("model_output_path", default_directory);

    // Create output directories
    std::string texture_directory = output_directory + "/materials/textures";
    std::string script_directory = output_directory + "/materials/scripts";
    std::string mesh_directory = output_directory + "/meshes";
    std::string model_directory = output_directory + "/models";
    boost::filesystem::create_directories(texture_directory);
    boost::filesystem::create_directories(script_directory);
    boost::filesystem::create_directories(mesh_directory);
    boost::filesystem::create_directories(model_directory);

    // Copy flat plane mesh to model directory
    std::string package_path = ros::package::getPath("aruco_markers_gazebo");
    boost::filesystem::copy_file(package_path + "/meshes/unit_plane.stl", mesh_directory + "/unit_plane.stl", boost::filesystem::copy_option::overwrite_if_exists);

    // Load template files into memory
    std::string model_template = load_file_to_string(package_path + "/models/marker_template.sdf");
    std::string material_template = load_file_to_string(package_path + "/materials/scripts/template.material");
    std::string config_template = load_file_to_string(package_path + "/models/marker_template.config");

    auto markers = input.find("markers");

    if (markers == input.end() || markers->type() != json::value_t::array)
    {
        ROS_WARN_NAMED(node_name, "No markers specified in input file, skipping generation");
        return 1;
    }

    for (auto i : *markers)
    {
        MarkerInfo marker = i.get<MarkerInfo>();
        std::string marker_name = fmt::format("{}_{}", dictionary_string, marker.id);

        std::string rendered_model = std::regex_replace(model_template, std::regex("\\[marker_name\\]"), marker_name);
        rendered_model = std::regex_replace(rendered_model, std::regex("\\[side_length\\]"), fmt::format("{}", marker.side_length));
        std::string rendered_material = std::regex_replace(material_template, std::regex("\\[marker_name\\]"), marker_name);
        std::string rendered_config = std::regex_replace(config_template, std::regex("\\[marker_name\\]"), marker_name);

        std::ofstream model_file(model_directory + "/" + marker_name + ".sdf");
        std::ofstream material_file(script_directory + "/" + marker_name + ".material");
        std::ofstream config_file(model_directory + "/" + marker_name + ".config");
        model_file << rendered_model;
        material_file << rendered_material;
        config_file << rendered_config;

        model_file.close();
        material_file.close();
        config_file.close();

        generate_aruco_image(dictionary_id, marker.id, texture_directory);
    }

    return 0;
}

bool generate_aruco_image(int dictionary_id, int marker_id, const std::string& directory)
{
    cv::Ptr<cv::aruco::Dictionary> dictionary_pointer = cv::aruco::getPredefinedDictionary(dictionary_id);
    std::string dictionary_name = lookup_dictionary_name_by_id(dictionary_id);

    std::string file_name = fmt::format("{}_{}.png", dictionary_name, marker_id);
    std::string file_path = fmt::format("{}/{}", directory, file_name);

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


#include "aruco_markers_gazebo/model_generator.h"

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>

#include <boost/filesystem.hpp>
#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include <fstream>
#include <stdlib.h>

using nlohmann::json;

void to_json(json& j, const MarkerInfo& marker)
{
    j = json{{"id", marker.id}, {"side_length", marker.side_length}};
}

void from_json(const json& j, MarkerInfo& marker)
{
    j.at("id").get_to(marker.id);
    j.at("side_length").get_to(marker.side_length);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "model_generator");
    ros::NodeHandle node("~");
    std::string input_path;

    if (!node.getParam("input_file", input_path))
    {
        ROS_FATAL_NAMED("model_generator", "No input file specified");
        return 1;
    }

    std::ifstream input_stream(input_path);

    if (!input_stream.is_open())
    {
        ROS_FATAL_NAMED("model_generator", "Cannot open input file %s", input_path.c_str());
        return 1;
    }

    json input;
    input_stream >> input;
    input_stream.close();

    ROS_WARN_COND_NAMED(!input.contains("dictionary"), "model_generator", "Input file %s does not specify a dictionary type - using default", input_path.c_str());

    std::string dictionary_string = input.value<std::string>("dictionary", "6x6");
    auto dictionary_result = get_dictionary_type_from_string(dictionary_string);

    ROS_WARN_COND_NAMED(!dictionary_result.first, "model_generator", "Malformatted dictionary type string - using default");
    DictionaryType dictionary_type = dictionary_result.second;

    std::string default_directory = std::string(getenv("HOME")) + "/.ros/gazebo_models/aruco";
    std::string output_directory = node.param<std::string>("model_output_path", default_directory);
    boost::filesystem::create_directories(output_directory);

    std::string texture_directory = output_directory + "/textures";
    std::string mesh_directory = output_directory + "/meshes";
    std::string model_directory = output_directory + "/models";
    boost::filesystem::create_directories(texture_directory);
    boost::filesystem::create_directories(mesh_directory);
    boost::filesystem::create_directories(model_directory);

    auto markers = input.find("markers");

    if (markers == input.end() || markers->type() != json::value_t::array)
    {
        ROS_WARN_NAMED("model_generator", "No markers specified in input file, skipping generation");
        return 1;
    }

    for (auto i : *markers)
    {
        MarkerInfo marker = i.get<MarkerInfo>();
        generate_aruco_image(dictionary_type, marker.id, texture_directory);
    }

    return 0;
}

bool generate_aruco_image(DictionaryType dictionary, unsigned char id, const std::string& directory)
{
    int dictionary_key;
    std::string dictionary_name;

    switch (dictionary)
    {
        case DictionaryType::Dict5x5:
            dictionary_key = cv::aruco::DICT_5X5_250;
            dictionary_name = "5x5";
            break;

        case DictionaryType::Dict6x6:
            dictionary_key = cv::aruco::DICT_6X6_250;
            dictionary_name = "6x6";
            break;

        case DictionaryType::Dict7x7:
            dictionary_key = cv::aruco::DICT_7X7_250;
            dictionary_name = "7x7";
            break;
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary_pointer = cv::aruco::getPredefinedDictionary(dictionary_key);
    std::string file_name = fmt::format("{}_{}.png", dictionary_name, id);
    std::string file_path = fmt::format("{}/{}", directory, file_name);

    cv::Mat marker_image;
    cv::aruco::drawMarker(dictionary_pointer, static_cast<int>(id), 1024, marker_image);

    boost::filesystem::create_directories(directory);
    if (!cv::imwrite(file_path, marker_image))
    {
        ROS_WARN_NAMED("model_generator", "Could not write marker image: %s", file_path.c_str());
        return false;
    }

    ROS_INFO_NAMED("model_generator", "Writing ArUco image to %s", file_path.c_str());
    return true;
}

std::pair<bool, DictionaryType> get_dictionary_type_from_string(const std::string& type)
{
    if (type == "5x5")
    {
        return {true, DictionaryType::Dict5x5};
    }
    else if (type == "6x6")
    {
        return {true, DictionaryType::Dict6x6};
    }
    else if (type == "7x7")
    {
        return {true, DictionaryType::Dict7x7};
    }

    return {false, DictionaryType::Dict6x6};
}

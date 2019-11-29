#pragma once

#include <string>
#include <utility>
#include <unordered_map>
#include <geometry_msgs/Pose.h>

extern std::unordered_map<std::string, int> dictionary_strings;

struct MarkerInfo
{
    int id;
    double side_length; // (in meters)
    geometry_msgs::Pose pose;
};

bool generate_aruco_image(int dictionary_id, int marker_id, const std::string& directory);

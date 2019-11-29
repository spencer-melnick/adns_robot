#pragma once

#include <string>
#include <utility>
#include <unordered_map>

extern std::unordered_map<std::string, int> dictionary_strings;

struct MarkerInfo
{
    int id;
    double side_length; // (in meters)
};

bool generate_aruco_image(int dictionary_id, int marker_id, const std::string& directory);

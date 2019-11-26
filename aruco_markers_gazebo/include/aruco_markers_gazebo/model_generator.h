#pragma once

#include <string>
#include <utility>

enum class DictionaryType
{
    Dict5x5,
    Dict6x6,
    Dict7x7
};

struct MarkerInfo
{
    unsigned char id;
    double side_length; // (in meters)
};

bool generate_aruco_image(DictionaryType dictionary, unsigned char id, const std::string& directory);
std::pair<bool, DictionaryType> get_dictionary_type_from_string(const std::string& type);

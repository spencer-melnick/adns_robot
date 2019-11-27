cmake_minimum_required(VERSION 2.8.3)
project(aruco_markers_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
)

find_package(Boost REQUIRED COMPONENTS
    filesystem
    system
)

find_package(OpenCV REQUIRED)
find_package(FMT REQUIRED)
find_package(nlohmann_json REQUIRED)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_model_generator src/model_generator.cpp)
set_target_properties(${PROJECT_NAME}_model_generator PROPERTIES OUTPUT_NAME model_generator PREFIX "")
add_dependencies(${PROJECT_NAME}_model_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_model_generator PUBLIC
    ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_model_generator PRIVATE
    ${OpenCV_LIBRARIES}
    fmt::fmt
    nlohmann_json::nlohmann_json
)


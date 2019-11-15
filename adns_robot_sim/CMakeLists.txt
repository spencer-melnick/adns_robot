cmake_minimum_required(VERSION 2.8.3)
project(adns_robot_sim)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    adns_robot_core
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS adns_robot_core
    DEPENDS roscpp
)

include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_differential_controller src/differential_controller.cpp)
add_dependencies(${PROJECT_NAME}_differential_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_differential_controller
    ${GAZEBO_LIBRARIES}
    ${catkin_LIBRARIES}
)

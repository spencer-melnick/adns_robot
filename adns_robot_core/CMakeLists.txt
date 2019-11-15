cmake_minimum_required(VERSION 2.8.3)
project(adns_robot_core)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
)

# find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
    FILES
    differential_velocity.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

# add_executable(${PROJECT_NAME}_node src/adns_robot_core_node.cpp)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
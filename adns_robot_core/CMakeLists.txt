cmake_minimum_required(VERSION 2.8.3)
project(adns_robot_core)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
    geometry_msgs
    roscpp
    tf2
    tf2_ros
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
    CATKIN_DEPENDS message_runtime std_msgs geometry_msgs roscpp
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_differential_drive src/differential_drive.cpp)
set_target_properties(${PROJECT_NAME}_differential_drive PROPERTIES OUTPUT_NAME differential_drive PREFIX "")
add_dependencies(${PROJECT_NAME}_differential_drive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_differential_drive
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_marker_tf_broadcaster src/marker_tf_broadcaster.cpp)
set_target_properties(${PROJECT_NAME}_marker_tf_broadcaster PROPERTIES OUTPUT_NAME marker_tf_broadcaster PREFIX "")
add_dependencies(${PROJECT_NAME}_marker_tf_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_marker_tf_broadcaster
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_pose_tf_broadcaster src/pose_tf_broadcaster.cpp)
set_target_properties(${PROJECT_NAME}_pose_tf_broadcaster PROPERTIES OUTPUT_NAME pose_tf_broadcaster PREFIX "")
add_dependencies(${PROJECT_NAME}_pose_tf_broadcaster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_pose_tf_broadcaster
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_tf_inverter src/tf_inverter.cpp)
set_target_properties(${PROJECT_NAME}_tf_inverter PROPERTIES OUTPUT_NAME tf_inverter PREFIX "")
add_dependencies(${PROJECT_NAME}_tf_inverter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_tf_inverter
  ${catkin_LIBRARIES}
)

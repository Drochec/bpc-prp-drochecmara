cmake_minimum_required(VERSION 3.20)

# Find the ROS2 GTest wrapper
find_package(ament_cmake_gtest REQUIRED)

# 1. Create the test executable.
# We explicitly compile just the test file and the kinematics implementation.
# This completely avoids pulling in your robot's main.cpp!
ament_add_gtest(t_kinematics
        t_kinematics.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../src/algorithms/kinematics.cpp
)

# 2. Tell the test where your header files live
target_include_directories(t_kinematics PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/algorithms
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/nodes
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/loops
)

# 3. Link your ROS2 dependencies (needed because t_kinematics includes motor.hpp)
ament_target_dependencies(t_kinematics
        rclcpp
        std_msgs
        sensor_msgs
        cv_bridge
        image_transport
        visualization_msgs
)

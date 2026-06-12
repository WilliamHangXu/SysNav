cmake_minimum_required(VERSION 3.10)
project(bag_slam_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(bag_slam_bridge src/bag_slam_bridge.cpp)
ament_target_dependencies(bag_slam_bridge
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros livox_ros_driver2)
target_link_libraries(bag_slam_bridge Eigen3::Eigen)

install(TARGETS bag_slam_bridge
  DESTINATION lib/${PROJECT_NAME})

# Python reference implementation, kept runnable as `bag_slam_bridge_py`.
install(PROGRAMS bag_slam_bridge/bag_slam_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bag_slam_bridge_py)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

ament_package()

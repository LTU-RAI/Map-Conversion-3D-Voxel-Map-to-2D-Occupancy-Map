cmake_minimum_required(VERSION 3.5)
project(mapconversion_msgs)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# Generate messages in the 'msg' folder
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HeightMap.msg"
  "msg/SlopeMap.msg"
  DEPENDENCIES std_msgs nav_msgs
)

ament_package()

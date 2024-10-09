cmake_minimum_required(VERSION 3.0.2)
project(mapconversion)


set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

## Find system dependencies
find_package(Eigen3 REQUIRED)

## Try to find UFOMap
find_package(ufomap QUIET)

## Try to find OctoMap
find_package(octomap QUIET)

find_package(ufomap_ros QUIET)
find_package(ufomap_msgs QUIET)
if(ufomap_ros_FOUND AND ufomap_msgs_FOUND)
  message(STATUS "Found UFOMap")
  set(USE_UFOMAP TRUE)
else()
  message(STATUS "UFOMap not found")
endif()

## Try to find OctoMap
find_package(octomap_ros QUIET)
find_package(octomap_msgs QUIET)
if(octomap_ros_FOUND  AND octomap_msgs_FOUND)
  message(STATUS "Found OctoMap")
  set(USE_OCTOMAP TRUE)
  #find_package(octomap QUIET)
else()
  message(STATUS "OctoMap not found")
endif()

if(NOT USE_UFOMAP AND NOT USE_OCTOMAP)
  message(FATAL_ERROR "Neither UFOMap nor OctoMap was found. Please install at least one of them.")
endif()

## Collect Catkin components
set(catkin_components
  roscpp
  roslib
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  message_generation
)

if(USE_UFOMAP)
  list(APPEND catkin_components ufomap_msgs ufomap_ros)
endif()

if(USE_OCTOMAP)
  list(APPEND catkin_components octomap_msgs octomap_ros)
endif()


## Find Catkin packages
find_package(catkin REQUIRED COMPONENTS ${catkin_components})



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  HeightMap.msg
  SlopeMap.msg
)

## Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)

## Catkin package configuration
## Catkin package configuration
set(CATKIN_DEPENDS_LIST
  message_runtime
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDS_LIST}
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

if(USE_UFOMAP)
  include_directories(${ufomap_INCLUDE_DIRS})
endif()

if(USE_OCTOMAP)
  include_directories(${OCTOMAP_INCLUDE_DIRS})
endif()


catkin_package()

## Build the library
add_library(map_convertion_calss src/MapConverter.cpp)
add_dependencies(map_convertion_calss ${grid_fast_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_convertion_calss ${catkin_LIBRARIES})

## Build nodes
if(USE_UFOMAP)
  add_executable(map_conversion_ufo_node src/map_conversion_ufo_node.cpp)
add_dependencies(map_conversion_ufo_node ${navigation_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  map_conversion_ufo_node
  UFO::Map
  map_convertion_calss
  ${catkin_LIBRARIES}
)
endif()

if(USE_OCTOMAP)
  add_executable(map_conversion_oct_node src/map_conversion_oct_node.cpp)
add_dependencies(map_conversion_oct_node ${navigation_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  map_conversion_oct_node
  map_convertion_calss
  ${catkin_LIBRARIES}
)
endif()

## Build the path converter node (common)
add_executable(path_converter_node src/path_converter_node.cpp)
add_dependencies(path_converter_node ${navigation_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  path_converter_node
  ${catkin_LIBRARIES}
)


cmake_minimum_required(VERSION 2.8.3)
project(rev_icp_mapper)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  libpointmatcher_ros
  roscpp
  rospy
  message_generation
  pcl_ros
  pcl_conversions
  tf
  nav_msgs
  octomap_mot
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(libpointmatcher REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  MatchCloudsREV.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rev_icp_mapper
  CATKIN_DEPENDS geometry_msgs sensor_msgs libpointmatcher_ros roscpp rospy pcl_ros  pcl_conversions tf nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(corrected_clouds_publisher src/corrected_clouds_publisher.cpp)
add_dependencies(corrected_clouds_publisher ${PROJECT_NAME}_gencpp)
target_link_libraries(corrected_clouds_publisher ${catkin_LIBRARIES} ${libpointmatcher_LIBRARIES})

add_executable(get_pose_node src/get_pose_node.cpp)
target_link_libraries(get_pose_node ${catkin_LIBRARIES})

add_executable(icp_odom_node src/icp_odom_node.cpp)
target_link_libraries(icp_odom_node ${catkin_LIBRARIES} ${libpointmatcher_LIBRARIES})

add_executable(rev_match_cloud_server src/rev_match_cloud_server.cpp)
add_dependencies(rev_match_cloud_server ${PROJECT_NAME}_gencpp)
target_link_libraries(rev_match_cloud_server ${catkin_LIBRARIES} ${libpointmatcher_LIBRARIES})

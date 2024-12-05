cmake_minimum_required(VERSION 3.0.2)
project(amr_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roslaunch
  rospy
  rospy_message_converter
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    rospy_message_converter
    sensor_msgs
    std_msgs
    tf
    visualization_msgs
)

#############
## Install ##
#############
catkin_python_setup()


catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

###########
## Build ##
###########

add_executable(obstacle_detection_node src/obstacle_detection_node.cpp)
target_link_libraries(obstacle_detection_node ${catkin_LIBRARIES})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
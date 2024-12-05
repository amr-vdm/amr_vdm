cmake_minimum_required(VERSION 3.0.2)
project(amr_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslaunch
)

catkin_package()

install(PROGRAMS
  mprim/genmprim_unicycle_highcost_5cm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  maps
  mprim
  params
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
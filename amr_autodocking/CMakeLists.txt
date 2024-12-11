cmake_minimum_required(VERSION 3.0.2)
project(amr_autodocking)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  geometry_msgs
  message_generation
  tf
)

# ## Add folders to be run by python nosetests
# # catkin_add_nosetests(test)
catkin_python_setup()

## Generate actions in the 'action' folder
add_action_files(
  FILES
  AutoDocking.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amr_autodocking
 CATKIN_DEPENDS actionlib_msgs geometry_msgs std_msgs message_runtime rospy roscpp
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
  scripts/autodock_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pattern_dock src/pattern_dock.cpp)
add_dependencies(pattern_dock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pattern_dock ${catkin_LIBRARIES})

add_executable(pattern_special_dock src/pattern_special_dock.cpp)
add_dependencies(pattern_special_dock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pattern_special_dock ${catkin_LIBRARIES})

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config
  launch
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

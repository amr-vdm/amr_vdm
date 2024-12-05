cmake_minimum_required(VERSION 3.0.2)
project(amr_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    AngularVelocities.msg
    AngularVelocitiesStamped.msg
    DockLimit.msg
    DockMode.msg
    DockParam.msg
    Encoders.msg
    EncodersStamped.msg
    Error.msg
    ErrorStamped.msg
    LightMode.msg
    WheelsCmd.msg
    WheelsCmdStamped.msg
    PID.msg
    PIDStamped.msg
    PoseInitial.msg
    CheckerSensorState.msg
    CheckerSensorStateStamped.msg
    SafetyStatus.msg
    SafetyStatusStamped.msg
    SafetyZone.msg
)

add_service_files(
  FILES
  ChangeFloor.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
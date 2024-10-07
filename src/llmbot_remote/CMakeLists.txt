cmake_minimum_required(VERSION 3.0.2)
project(llmbot_remote)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  message_generation
  moveit_ros_planning_interface
)

## Declare the message and service files
add_action_files(
  FILES
  llmbot_task.action
  llmbot_gripper.action
  llmbot_arm.action
)

add_service_files(
  FILES
  SceneDescription.srv
)

## Generate the messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(
  PROGRAMS
  scripts/task_server.py
  scripts/gripper_server.py
  scripts/arm_server.py
  scripts/scene_description_service.py  # Add any additional Python scripts here
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Declare a catkin package and specify dependencies
catkin_package(
  CATKIN_DEPENDS 
    roscpp
    rospy
    std_msgs
    message_runtime  # Include message_runtime for the generated messages and services
    actionlib
)

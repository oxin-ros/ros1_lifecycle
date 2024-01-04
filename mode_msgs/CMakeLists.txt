cmake_minimum_required(VERSION 2.8.3)
project(mode_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation actionlib_msgs)

add_message_files(
  FILES
  Mode.msg
)

add_service_files(
  FILES
  ListModes.srv
)

add_action_files(
  FILES
  ModeChange.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime
)

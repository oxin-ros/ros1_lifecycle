cmake_minimum_required(VERSION 2.8.3)
project(mode_manager)

find_package(catkin REQUIRED COMPONENTS mode_msgs lifecycle_python lifecycle_msgs)

catkin_package(
    CATKIN_DEPENDS lifecycle_msgs mode_msgs
)

install(
    PROGRAMS
    scripts/change_mode.py
    src/check_config.py
    src/manager_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

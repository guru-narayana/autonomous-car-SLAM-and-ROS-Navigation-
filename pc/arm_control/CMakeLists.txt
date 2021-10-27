cmake_minimum_required(VERSION 3.0.2)
project(arm_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  controller_manager
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs
# DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
add_executable(MyRobot_hardware_interface src/MyRobot_hardware_interface.cpp)
target_link_libraries(MyRobot_hardware_interface ${catkin_LIBRARIES})
add_dependencies(MyRobot_hardware_interface ${catkin_EXPORTED_TARGETS})
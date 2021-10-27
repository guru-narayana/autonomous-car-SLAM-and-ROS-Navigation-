cmake_minimum_required(VERSION 3.0.2)
project(bot_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  actionlib_msgs
)

 add_message_files(
  FILES
  servo.msg
  feedback.msg
 )

 add_service_files(
   FILES
   Inverse_kinematics.srv
 )

 add_action_files(
   FILES
   joint_control.action
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
 )
catkin_package(
CATKIN_DEPENDS message_runtime
)


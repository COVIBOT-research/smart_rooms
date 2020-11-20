cmake_minimum_required(VERSION 2.8.3)
project(smart_rooms_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   RoomStamped.msg
   RoomStampedList.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   nav_msgs
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs std_msgs
)

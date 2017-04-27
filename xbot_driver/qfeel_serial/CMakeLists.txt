cmake_minimum_required(VERSION 2.8.3)
project(qfeel_serial)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 rospy
 tf
 std_msgs
 nav_msgs
 message_generation
) 

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  imu.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES qfeel_serial
 CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs message_runtime
 DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include)

## Declare a C++ executable
add_executable(qfeel_serial src/qfeel_serial.cpp src/serial_port.cpp src/qfeel_sub_pub.cpp src/qfeel_pack.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(qfeel_serial
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


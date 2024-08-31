cmake_minimum_required(VERSION 3.0.2)
project(spice_up_coordinator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
  actionlib
)

## Find Boost
find_package(Boost REQUIRED COMPONENTS system thread)

add_action_files(DIRECTORY action FILES SpiceUpBottlePick.action)

add_service_files(
   FILES
   SpiceName.srv
   IDXAcquisition.srv
)

generate_messages(
  DEPENDENCIES std_msgs actionlib_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs actionlib roscpp
  DEPENDS Boost
  )

catkin_install_python(PROGRAMS 
  scripts/main.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/spice_up_coordinator
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


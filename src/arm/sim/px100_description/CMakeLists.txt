cmake_minimum_required(VERSION 2.8.3)
project(px100_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  joint_state_publisher_gui
  robot_state_publisher
  roslaunch
  rviz
  xacro
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS 
  joint_state_publisher 
  joint_state_publisher_gui 
  robot_state_publisher 
  roslaunch 
  rviz 
  xacro
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
    launch
    meshes
    rviz
    urdf
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

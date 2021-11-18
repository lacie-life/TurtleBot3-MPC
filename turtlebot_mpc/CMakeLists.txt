#CMakeLists.txt
cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   geometry_msgs
   std_msgs
   gazebo_msgs
   tf
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlebot_mpc
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# install(DIRECTORY launch
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(DIRECTORY cfg
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

catkin_install_python(PROGRAMS
   scripts/turtlebot_mpc_node
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


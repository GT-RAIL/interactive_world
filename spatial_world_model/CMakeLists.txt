cmake_minimum_required(VERSION 2.8.3)
project(spatial_world_model)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rail_manipulation_msgs roscpp interactive_world_msgs ar_track_alvar_msgs roslib tf2 tf2_ros geometry_msgs std_srvs)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mysqlclient
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(spatial_world_model_server src/spatial_world_model_server.cpp)
add_executable(recognized_objects_listener
  nodes/recognized_objects_listener.cpp
  src/RecognizedObjectsListener.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(spatial_world_model_server
  ${catkin_LIBRARIES}
  mysqlclient
)
target_link_libraries(recognized_objects_listener
  ${catkin_LIBRARIES}
)

add_dependencies(spatial_world_model_server interactive_world_msgs_gencpp)
add_dependencies(recognized_objects_listener interactive_world_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS spatial_world_model_server recognized_objects_listener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

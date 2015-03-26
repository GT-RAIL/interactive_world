cmake_minimum_required(VERSION 2.8.3)
project(interactive_world_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  carl_dynamixel
  carl_moveit
  interactive_world_msgs
  geometry_msgs
  move_base_msgs
  roscpp
  roslib
  std_srvs
  tf2
  tf2_ros
)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PkgConfig)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Check for what version of YAML we have
pkg_check_modules(YAML_CPP yaml-cpp)
if(${YAML_CPP_VERSION} VERSION_GREATER 0.5)
  add_definitions(-DYAMLCPP_GT_0_5_0)
endif()

## Specify additional locations of header files
include_directories(include
  ${boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

## Declare a cpp executable
add_executable(interactive_world_model_fetcher nodes/interactive_world_model_fetcher.cpp)
add_executable(high_level_action_server
  nodes/high_level_action_server.cpp
  src/HighLevelActions.cpp
  src/PlacementSurface.cpp
  src/PointOfInterest.cpp
  src/Room.cpp
  src/Surface.cpp
  src/World.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(interactive_world_model_fetcher
  ${catkin_LIBRARIES}
  jsoncpp
  curl
)
target_link_libraries(high_level_action_server
  ${boost_LIBRARIES}
  ${catkin_LIBRARIES}
  yaml-cpp
)

## Add message build dependencies (needed for source build)
add_dependencies(interactive_world_model_fetcher interactive_world_msgs_gencpp)
add_dependencies(high_level_action_server interactive_world_msgs_gencpp wpi_jaco_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS interactive_world_model_fetcher high_level_action_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

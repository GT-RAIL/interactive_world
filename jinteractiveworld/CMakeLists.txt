cmake_minimum_required(VERSION 2.8.3)
project(jinteractiveworld)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

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

execute_process(
  COMMAND mvn assembly:assembly
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/
)

#############
## Install ##
#############

## Copy scripts as executables
install(PROGRAMS
  scripts/interactive_world_learner
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy JAR files
install(FILES target/${PROJECT_NAME}-ROS-SNAPSHOT-jar-with-dependencies.jar
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/target
)

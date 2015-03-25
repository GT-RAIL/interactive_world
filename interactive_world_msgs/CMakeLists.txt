cmake_minimum_required(VERSION 2.8.3)
project(interactive_world_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(FILES
  Configuration.msg
  Item.msg
  Model.msg
  Placement.msg
  PlacementSet.msg
  PointOfInterest.msg
  Room.msg
  Surface.msg
  TaskModels.msg
  TaskTrainingData.msg
)

## Generate services in the 'srv' folder
add_service_files(FILES
  LearnModels.srv
  LoadModels.srv
  Parse.srv
  StoreObservation.srv
)

## Generate actions in the 'action' folder
add_action_files(FILES
  DriveAndSearch.action
  DriveToSurface.action
  ObjectSearch.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

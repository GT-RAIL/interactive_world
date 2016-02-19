#ifndef INTERACTIVE_WORLD_RECOGNIZED_OBJECTS_LISTENER_H_
#define INTERACTIVE_WORLD_RECOGNIZED_OBJECTS_LISTENER_H_

#include <ros/ros.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <tf2_ros/transform_listener.h>

namespace rail
{
namespace interactive_world
{

class RecognizedObjectsListener
{
public:
  RecognizedObjectsListener();

private:
  /*! The global ROS node handle. */
  ros::NodeHandle node_;
  /*! The recognized objects topic. */
  ros::Subscriber recognized_objects_sub_;
  /*! The store observation service and surface finder. */
  ros::ServiceClient store_observation_srv_, find_surface_srv_, transform_to_surface_frame_srv_,
      find_observations_srv_, set_observations_removed_srv_;

  bool incoming_cleared_objects_;

  void recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &objects);
};

}
}

#endif

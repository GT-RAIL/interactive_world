#ifndef INTERACTIVE_WORLD_RECOGNIZED_OBJECTS_LISTENER_H_
#define INTERACTIVE_WORLD_RECOGNIZED_OBJECTS_LISTENER_H_

#include <ros/ros.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <interactive_world_msgs/StoreObservation.h>

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
  /*! The store observation service. */
  ros::ServiceClient store_observation_srv_;

  void recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &objects);
};

}
}

#endif

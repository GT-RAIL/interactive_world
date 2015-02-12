#include <spatial_world_model/Tracker.h>

using namespace std;
using namespace spatial_world_model;

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "spatial_world_model_tracker");
  Tracker tracker;
  ros::spin();
}

#include <spatial_world_model/RecognizedObjectsListener.h>

using namespace std;
using namespace rail::interactive_world;

/*!
 * Creates and runs the recognized_objects_listener node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "recognized_objects_listener");
  RecognizedObjectsListener listener;
  ros::spin();
  return EXIT_SUCCESS;
}

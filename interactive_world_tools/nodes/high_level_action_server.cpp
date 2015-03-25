/*!
 * \file high_level_action_server.cpp
 * \brief The interactive world high level action server.
 *
 * The high level action server contains high level functionality for doing things such as driving to surfaces.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#include <interactive_world_tools/HighLevelActions.h>

using namespace std;
using namespace rail::interactive_world;

/*!
 * Creates and runs the high_level_action_server node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly or EXIT_FAILURE if an error occurs.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "high_level_action_server");
  HighLevelActions actions;
  // check if everything started okay
  if (actions.okay())
  {
    ros::spin();
    return EXIT_SUCCESS;
  } else
  {
    return EXIT_FAILURE;
  }
}

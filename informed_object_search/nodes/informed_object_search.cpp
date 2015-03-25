/*!
 * \file informed_object_search.cpp
 * \brief The main object search node.
 *
 * The object searcher node is responsible for searching for a given object given information obtained from the
 * interactive world project. Associated ROS information is maintained in this class.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#include <informed_object_search/ObjectSearcher.h>

using namespace std;
using namespace rail::interactive_world;

/*!
 * Creates and runs the informed_object_search node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "informed_object_search");
  ObjectSearcher searcher;
  ros::spin();
  return EXIT_SUCCESS;
}

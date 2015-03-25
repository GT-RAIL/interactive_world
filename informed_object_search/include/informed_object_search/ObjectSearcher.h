/*!
 * \file ObjectSearcher.h
 * \brief The main object search node class.
 *
 * The object searcher node is responsible for searching for a given object given information obtained from the
 * interactive world project. Associated ROS information is maintained in this class.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#ifndef INTERACTIVE_WORLD_OBJECT_SEARCHER_H_
#define INTERACTIVE_WORLD_OBJECT_SEARCHER_H_

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <interactive_world_msgs/ObjectSearchAction.h>
#include <interactive_world_msgs/DriveAndSearchAction.h>
#include <ros/ros.h>

namespace rail
{
namespace interactive_world
{

/*!
 * \class ObjectSearcher
 * \brief The main search and retrieve node class.
 *
 * The object searcher node is responsible for searching for a given object given information obtained from the
 * interactive world project. Associated ROS information is maintained in this class.
 */
class ObjectSearcher
{
public:
  /*! The default wait time for action servers in seconds. */
  static const int AC_WAIT_TIME = 120;

  /*!
   * \brief Create a ObjectSearcher and associated ROS information.
   *
   * Creates a ROS node handle, subscribes to the relevant topics and servers, and creates interfaces for requesting
   * object searches.
   */
  ObjectSearcher();

private:
  /*!
   * \brief Callback for the object search action server.
   *
   * The search action will search the world for the given object using data collected from the interactive world.
   *
   * \param goal The goal object specifying the parameters.
   */
  void objectSearch(const interactive_world_msgs::ObjectSearchGoalConstPtr &goal);

  /*! The global and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The main object search action server. */
  actionlib::SimpleActionServer<interactive_world_msgs::ObjectSearchAction> as_;
  /*! The drive and search action client. */
  actionlib::SimpleActionClient<interactive_world_msgs::DriveAndSearchAction> search_ac_;
  /*! The model fetcher service client. */
  ros::ServiceClient model_fetcher_srv_;
  /*! The action client timeout. */
  ros::Duration ac_wait_time_;
};

}
}

#endif

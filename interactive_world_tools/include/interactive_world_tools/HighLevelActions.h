/*!
 * \file HighLevelActions.h
 * \brief The interactive world high level action server.
 *
 * The high level action server contains high level functionality for doing things such as driving to surfaces.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#ifndef INTERACTIVE_WORLD_HIGH_LEVEL_ACTIONS_H_
#define INTERACTIVE_WORLD_HIGH_LEVEL_ACTIONS_H_

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/TransformStamped.h>
#include <interactive_world_msgs/DriveAndSearchAction.h>
#include <interactive_world_msgs/DriveToSurfaceAction.h>
#include <interactive_world_tools/World.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
//#include <wpi_jaco_msgs/HomeArmAction.h>

#include <boost/thread/mutex.hpp>

namespace rail
{
namespace interactive_world
{

/*!
 * \class HighLevelActions
 * \brief The interactive world high level action server.
 *
 * The high level action server contains high level functionality for doing things such as driving to surfaces.
 */
class HighLevelActions
{
public:
  /*! The default wait time for action servers in seconds. */
  static const int AC_WAIT_TIME = 60;
  /*! Constant used for the W and Z components of a quaternion for a 90 degree rotation about the Z axis. */
  static const double QUATERNION_90_ROTATE = 0.7071067811865476;

  /*!
   * \brief Create a HighLevelActions and associated ROS information.
   *
   * Creates a ROS node handle, subscribes to the relevant topics and servers, and creates interfaces for the
   * different high level actions.
   */
  HighLevelActions();

  /*!
   * \brief A check for a valid HighLevelActions.
   *
   * This function will return true if valid parameters were parsed from a YAML config file.
   *
   * \return True if valid parameters were parsed.
   */
  bool okay() const;

private:
  /*!
   * \brief Callback for the object search action server.
   *
   * The search action will search for a given object at a given surface in the world.
   *
   * \param goal The goal object specifying the parameters.
   */
  void driveAndSearch(const interactive_world_msgs::DriveAndSearchGoalConstPtr &goal);

  /*!
   * \brief Callback for the drive to surface action server.
   *
   * The drive action will attempt to drive to a pre-defined naviagtion goal at a given surface.
   *
   * \param goal The goal object specifying the parameters.
   */
  void driveToSurface(const interactive_world_msgs::DriveToSurfaceGoalConstPtr &goal);

  /*!
   * \brief Navigation helper.
   *
   * Attempt to drive to the given frame.
   *
   * \param frame_id The frame ID to drive to.
   * \return True if the navigation was successful.
   */
  bool driveHelper(const std::string &frame_id);

  /*!
   * \brief Create a new tf2 transform from a ROS message.
   *
   * Creates and returns a new TF2 transform object from a ROS message.
   *
   * \param tf The ROS transform message to take values from.
   * \return The new tf2 transform.
   */
  tf2::Transform tfFromTFMessage(const geometry_msgs::Transform &tf);

  /*! The okay check flags. */
  bool debug_, okay_;
  /*! The global and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The world configuration. */
  World world_;
  /*! Static transforms in the world. */
  std::map<std::string, geometry_msgs::TransformStamped> static_tfs_;
  /*! The object search action server. */
  actionlib::SimpleActionServer<interactive_world_msgs::DriveAndSearchAction> drive_and_search_as_;
  /*! The drive action server. */
  actionlib::SimpleActionServer<interactive_world_msgs::DriveToSurfaceAction> drive_to_surface_as_;
  /*! The navigation action client. */
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> nav_ac_;
  /*! The home arm action client. */
  //actionlib::SimpleActionClient<wpi_jaco_msgs::HomeArmAction> home_arm_ac_;
  /*! The camera and segmentation service client. */
  ros::ServiceClient look_at_frame_srv_, segment_srv_;
  /*! The action client timeout. */
  ros::Duration ac_wait_time_;
  /*! The drive action client mutex */
  boost::mutex nav_mutex_;
};

}
}

#endif

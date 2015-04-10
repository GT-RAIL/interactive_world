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
#include <carl_moveit/ArmAction.h>
#include <geometry_msgs/TransformStamped.h>
#include <interactive_world_msgs/DriveAndSearchAction.h>
#include <interactive_world_msgs/DriveToSurfaceAction.h>
#include <interactive_world_msgs/FindSurface.h>
#include <interactive_world_msgs/TransformToSurfaceFrame.h>
#include <interactive_world_tools/World.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>

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
  /*! Threshold on the Z axis for something being on a surface. */
  static const double SURFACE_Z_THRESH = 0.1;

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
   * \brief Callback for the find surface server.
   *
   * Searches for the surface in the world that the given point is on (within some threshold).
   *
   * \param req The request with the point to check.
   * \param resp The response with the surface name and frame (if any).
   * \return True if the call was successful.
   */
  bool findSurfaceCallback(interactive_world_msgs::FindSurface::Request &req,
      interactive_world_msgs::FindSurface::Response &resp);

  bool transformToSurfaceFrame(interactive_world_msgs::TransformToSurfaceFrame::Request &req,
      interactive_world_msgs::TransformToSurfaceFrame::Response &resp);

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

  /*!
   * \brief Create a new tf2 transform from a ROS message.
   *
   * Creates and returns a new TF2 transform object from a ROS message.
   *
   * \param pose The ROS pose message to take values from.
   * \return The new tf2 transform.
   */
  tf2::Transform tfFromPoseMessage(const geometry_msgs::Pose &pose);

  /*!
   * \brief Store the latest recognized objects.
   *
   * Stores the latest recognized objects from the segmentation topic.
   *
   * \param objects The latest recognized objects.
   */
  void recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &objects);

  /*! The okay check flags. */
  bool debug_, okay_;
  /*! The global and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! The fixed frame. */
  std::string fixed_frame_;
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
  actionlib::SimpleActionClient<carl_moveit::ArmAction> home_arm_ac_;
  /*! The camera and segmentation service clients. */
  ros::ServiceClient look_at_frame_srv_, segment_srv_;
  /*! The find surface server. */
  ros::ServiceServer find_surface_srv_, transform_to_surface_frame_srv_;
  /*! The recognized objects topic. */
  ros::Subscriber recognized_objects_sub_;
  /*! The action client timeout. */
  ros::Duration ac_wait_time_;
  /*! The various mutexes used. */
  boost::mutex nav_mutex_, recognized_objects_mutex_;
  /*! The latest recognized objects */
  rail_manipulation_msgs::SegmentedObjectList recognized_objects_;
  /*! A counter for the number of recognized object messages that have come in. */
  uint32_t recognized_objects_counter_;
};

}
}

#endif

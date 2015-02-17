#ifndef SPATIAL_WORLD_MODEL_TRACKER_H_
#define SPATIAL_WORLD_MODEL_TRACKER_H_

#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <std_srvs/Empty.h>
#include <boost/thread/mutex.hpp>
#include <spatial_world_model/Surface.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <map>
#include <tf2/LinearMath/Transform.h>

namespace spatial_world_model
{

class Tracker
{
public:
  static const int TF_CACHE_TIME = 5;
  static const double SURFACE_Z_THRESH = 0.05;

  Tracker();

private:
  void markerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &markers);

  bool snapshotCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

  tf2::Transform tfFromTFMessage(geometry_msgs::Transform &msg);

  tf2::Transform tfFromPoseMessage(geometry_msgs::Pose &msg);

  ros::NodeHandle node_;
  ros::Subscriber ar_pose_marker_;
  ros::ServiceServer snapshot_;
  ros::ServiceClient store_;
  std::vector<Surface> surfaces_;
  std::vector<ar_track_alvar_msgs::AlvarMarker> latest_markers_;
  std::map<uint32_t, std::string> mappings_;
  std::map<std::string, geometry_msgs::TransformStamped> static_tfs_;
  std::string fixed_frame_;
  boost::mutex mutex_;
};

}

#endif

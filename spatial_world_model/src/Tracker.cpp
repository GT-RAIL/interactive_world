#include <spatial_world_model/Tracker.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <tf2_ros/transform_listener.h>
#include <interactive_world_msgs/StoreObservation.h>

using namespace std;
using namespace spatial_world_model;

Tracker::Tracker()
{
  ros::NodeHandle private_node("~");

  // grab YAML files
  string mapping_file;
  private_node.param("mapping_config", mapping_file, ros::package::getPath("spatial_world_model") + "/config/mapping.yaml");
  string surface_frames_file;
  private_node.param("surface_frames", surface_frames_file, ros::package::getPath("spatial_world_model") + "/config/surface_frames.yaml");

  // setup topics and services
  ar_pose_marker_ = node_.subscribe("/ar_pose_marker", 1, &Tracker::markerCallback, this);
  snapshot_ = private_node.advertiseService("snapshot", &Tracker::snapshotCallback, this);
  store_ = node_.serviceClient<interactive_world_msgs::StoreObservation>("/spatial_world_model_server/store_observation");

  ROS_INFO("Spatial World Model Tracker Initialized");
}

void Tracker::markerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &markers)
{
  // lock for the vector
  boost::mutex::scoped_lock lock(mutex_);

  // clear out old cache
  latest_markers_.clear();

  // store any markers that are in our YAML file
  for (size_t i = 0; i < markers->markers.size(); i++)
  {
    if (mappings_.find(markers->markers[i].id) != mappings_.end())
    {
      latest_markers_.push_back(markers->markers[i]);
    } else
    {
      ROS_WARN("Marker #%d not in mappings configuration.", markers->markers[i].id);
    }
  }
}

bool Tracker::snapshotCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  // lock for the vector
  boost::mutex::scoped_lock lock(mutex_);

  if (latest_markers_.size() == 0)
  {
    ROS_WARN("No markers detected at time of snapshot.");
  } else
  {
    // go through each object we have seen
    int item_count = 0;
    for (size_t i = 0; i < latest_markers_.size(); i++)
    {
      // get the TF of the marker in the world
      ar_track_alvar_msgs::AlvarMarker &m = latest_markers_[i];
      tf2::Transform t_marker_world = this->tfFromPoseMessage(m.pose.pose);

      // determine which object this is
      string &object = mappings_[m.id];

      bool surface_found = false;
      // check for the each placement surface -- loop breaks on surface_found == true
      for (size_t j = 0; j < surfaces_.size() && !surface_found; j++)
      {
        // transform the marker into the surface frame
        Surface &s = surfaces_[j];
        tf2::Transform t_surface_world = this->tfFromTFMessage(static_tfs_[s.getFrame()].transform);
        tf2::Transform t_marker_surface = t_surface_world.inverseTimes(t_marker_world);

        // first check if we are inside that surface's bounding box
        if ((s.getWidth() / 2.0 >= abs(t_marker_surface.getOrigin().x())) && (s.getHeight() / 2.0 >= abs(t_marker_surface.getOrigin().y())))
        {
          double closest = numeric_limits<double>::infinity();

          // used to store an observation if a placement surface is found
          interactive_world_msgs::StoreObservation req;
          req.request.surface = s.getName();
          req.request.item = mappings_[m.id];
          req.request.item_conf = 1.0;

          // now check if we are close to the actual surface
          for (size_t k = 0; k < s.getNumPlacementSurfaces(); k++)
          {
            tf2::Transform t_placement_surface_world = this->tfFromTFMessage(static_tfs_[s.getPlacementSurface(k)].transform);
            double surface_z = t_placement_surface_world.getOrigin().z();
            double marker_z = m.pose.pose.position.z;
            if (abs(surface_z - marker_z) <= SURFACE_Z_THRESH)
            {
              surface_found = true;
              // used to find closest placement surface
              tf2::Transform t_marker_placement_surface = t_placement_surface_world.inverseTimes(t_marker_world);
              double dist = t_marker_placement_surface.getOrigin().distance(tf2::Vector3(0, 0, 0));
              if (dist < closest)
              {
                closest = dist;
                req.request.placement_surface_frame_id = s.getPlacementSurface(k);
                req.request.x = t_marker_placement_surface.getOrigin().x();
                req.request.y = t_marker_placement_surface.getOrigin().y();
                req.request.z = t_marker_placement_surface.getOrigin().z();
                req.request.theta = t_marker_placement_surface.getRotation().getAngle();
              }
            }
          }

          if (surface_found)
          {
            // store the observation
            item_count++;
            store_.call(req);
          }
        }
      }

      // check for a warning
      if (!surface_found)
      {
        ROS_WARN("Could not determine which surface '%s' is on.", object.c_str());
      }
    }

    ROS_INFO("Stored %i observations.", item_count);
  }

  return true;
}

tf2::Transform Tracker::tfFromTFMessage(geometry_msgs::Transform &msg)
{
  // construct and return
  return tf2::Transform(tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w),
      tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));
}

tf2::Transform Tracker::tfFromPoseMessage(geometry_msgs::Pose &msg)
{
  // construct and return
  return tf2::Transform(tf2::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
      tf2::Vector3(msg.position.x, msg.position.y, msg.position.z));
}

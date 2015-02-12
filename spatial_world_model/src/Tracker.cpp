#include <spatial_world_model/Tracker.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

using namespace std;
using namespace spatial_world_model;

Tracker::Tracker()
{
  ros::NodeHandle private_node("~");

  // grab params
  private_node.param("fixed_frame", fixed_frame_, string("map"));

  // grab YAML files
  string mapping_file;
  private_node.param("mapping_config", mapping_file, ros::package::getPath("spatial_world_model") + "/config/mapping.yaml");
  string surface_frames_file;
  private_node.param("surface_frames", surface_frames_file, ros::package::getPath("spatial_world_model") + "/config/surface_frames.yaml");

#ifdef YAMLCPP_GT_0_5_0
  ROS_INFO("Parsing spatial world model tracking configurations...");

  // used to gather static TFs
  ros::Duration cache_time(TF_CACHE_TIME);
  tf2_ros::Buffer tf_buffer(cache_time);
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // parse the object/marker mappings
  YAML::Node mapping_config = YAML::LoadFile(mapping_file);
  for (size_t i = 0; i < mapping_config.size(); i++)
  {
    // extract the mapping
    mappings_[mapping_config[i]["marker_id"].as<uint32_t>()] = mapping_config[i]["object_name"].as<string>();
  }

  // parse the surface frame groundings
  YAML::Node surface_frames_config = YAML::LoadFile(surface_frames_file);
  for (size_t i = 0; i < surface_frames_config.size(); i++)
  {
    // extract the frame information
    YAML::Node cur = surface_frames_config[i];
    Surface surface(cur["name"].as<string>(), cur["frame"].as<string>(), cur["width"].as<double>(), cur["height"].as<double>());
    // get the static TF information
    static_tfs_[surface.getFrame()] = tf_buffer.lookupTransform(fixed_frame_, surface.getFrame(), ros::Time(0), ros::Duration(TF_CACHE_TIME + 1));
    // grab placement surfaces
    YAML::Node placement_surfaces = cur["placement_surfaces"];
    for (size_t j = 0; j < placement_surfaces.size(); j++)
    {
      // store the placement surface
      surface.addPlacementSurface(placement_surfaces[j]["frame"].as<string>());
      // get the static TF information
      static_tfs_[surface.getPlacementSurface(j)] = tf_buffer.lookupTransform(fixed_frame_, surface.getPlacementSurface(j), ros::Time(0), ros::Duration(TF_CACHE_TIME + 1));
    }
    // grab POIs
    YAML::Node pois = cur["pois"];
    for (size_t j = 0; j < pois.size(); j++)
    {
      // store the POI
      surface.addPOI(PointOfInterest(pois[j]["name"].as<string>(), pois[j]["frame"].as<string>()));
      // get the static TF information
      static_tfs_[surface.getPOI(j).getFrame()] = tf_buffer.lookupTransform(fixed_frame_, surface.getPOI(j).getFrame(), ros::Time(0), ros::Duration(TF_CACHE_TIME + 1));
    }
    surfaces_.push_back(surface);

    // get the static TF information
    static_tfs_[surface.getFrame()] = tf_buffer.lookupTransform(fixed_frame_, surface.getFrame(), ros::Time(0), ros::Duration(TF_CACHE_TIME + 1));
  }
#else
  ROS_WARN("WARNING: Unsupported version of YAML. Config files not parsed.");
#endif

  // setup topics and services
  ar_pose_marker_ = node_.subscribe("/ar_pose_marker", 1, &Tracker::markerCallback, this);
  snapshot_ = private_node.advertiseService("snapshot", &Tracker::snapshotCallback, this);

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
    for (size_t i = 0; i < latest_markers_.size(); i++)
    {
      // get the TF of the marker in the world
      ar_track_alvar_msgs::AlvarMarker &m = latest_markers_[i];
      tf2::Transform t_marker_world = this->tfFromPoseMessage(m.pose.pose);

      // determine which object this is
      string &object = mappings_[m.id];

      // check for the each placement surface
      bool surface_found = false;
      for (size_t j = 0; j < surfaces_.size() && !surface_found; j++)
      {
        // transform the marker into the surface frame
        Surface &s = surfaces_[j];
        tf2::Transform t_surface_world = this->tfFromTFMessage(static_tfs_[s.getFrame()].transform);
        tf2::Transform t_marker_surface = t_surface_world.inverseTimes(t_marker_world);

        // first check if we are inside that surface's bounding box
        if ((s.getWidth() / 2.0 >= abs(t_marker_surface.getOrigin().x())) && (s.getHeight() / 2.0 >= abs(t_marker_surface.getOrigin().y())))
        {
          // now check if we are close to the actual surface
          for (size_t k = 0; k < s.getNumPlacementSurfaces() && !surface_found; k++)
          {
            double surface_z = static_tfs_[s.getPlacementSurface(k)].transform.translation.z;
            double marker_z = m.pose.pose.position.z;
            if (abs(surface_z - marker_z) <= SURFACE_Z_THRESH)
            {
              surface_found = true;
              ROS_INFO("'%s' is on '%s'", object.c_str(), s.getName().c_str());
            }
          }
        }
      }

      // check for a warning
      if (!surface_found)
      {
        ROS_WARN("Could not determine which surface '%s' is on.", object.c_str());
      }
    }
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

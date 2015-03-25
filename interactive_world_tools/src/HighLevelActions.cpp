/*!
 * \file HighLevelActions.cpp
 * \brief The interactive world high level action server.
 *
 * The high level action server contains high level functionality for doing things such as driving to surfaces.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#include <boost/algorithm/string.hpp>
#include <carl_dynamixel/LookAtFrame.h>
#include <interactive_world_tools/HighLevelActions.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace rail::interactive_world;

HighLevelActions::HighLevelActions()
    : private_node_("~"), nav_ac_("/move_base", true), ac_wait_time_(AC_WAIT_TIME),
      drive_and_search_as_(
          private_node_, "drive_and_search", boost::bind(&HighLevelActions::driveAndSearch, this, _1), false
      ),
      drive_to_surface_as_(
          private_node_, "drive_to_surface", boost::bind(&HighLevelActions::driveToSurface, this, _1), false
      )
{
  // set defaults
  string world_file(ros::package::getPath("interactive_world_tools") + "/config/world.yaml");
  string fixed_frame("map");

  // grab any parameters we need
  private_node_.getParam("world_config", world_file);
  private_node_.getParam("fixed_frame", fixed_frame);

  // setup services and topic
  look_at_frame_srv_ = node_.serviceClient<carl_dynamixel::LookAtFrame>("/asus_controller/look_at_frame");
  segment_srv_ = node_.serviceClient<std_srvs::Empty>("/rail_segmentation/segment");

// check the YAML version
#ifdef YAMLCPP_GT_0_5_0
  // used to gather static TFs
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  // give the buffer time to fill
  sleep(tf_buffer.getCacheLength().sec / 10);

  // parse the world config
  YAML::Node world_config = YAML::LoadFile(world_file);
  for (size_t i = 0; i < world_config.size(); i++)
  {
    // starts with rooms
    YAML::Node room_config = world_config[i];
    Room room(room_config["name"].as<string>(), room_config["frame_id"].as<string>());
    // get our TF information
    static_tfs_[room.getFrameID()] = tf_buffer.lookupTransform(fixed_frame, room.getFrameID(), ros::Time(0));

    // get each surface
    YAML::Node surfaces_config = room_config["surfaces"];
    for (size_t j = 0; j < surfaces_config.size(); j++)
    {
      YAML::Node surface_config = surfaces_config[j];
      // parse the name, frame, and size
      Surface surface(surface_config["name"].as<string>(), surface_config["frame_id"].as<string>(),
          surface_config["width"].as<double>(), surface_config["height"].as<double>());
      // get our TF information
      static_tfs_[surface.getFrameID()] = tf_buffer.lookupTransform(fixed_frame, surface.getFrameID(), ros::Time(0));

      // get each alias
      YAML::Node aliases_config = surface_config["aliases"];
      for (size_t k = 0; k < aliases_config.size(); k++)
      {
        // parse the name
        surface.addAlias(aliases_config[k].as<string>());
      }

      // get each placement surface
      YAML::Node placement_surfaces_config = surface_config["placement_surfaces"];
      for (size_t k = 0; k < placement_surfaces_config.size(); k++)
      {
        // parse the frames
        YAML::Node ps_config = placement_surfaces_config[k];
        PlacementSurface ps(ps_config["frame_id"].as<string>(), ps_config["nav_frame_id"].as<string>());
        surface.addPlacementSurface(ps);
        // get our TF information
        static_tfs_[ps.getFrameID()] = tf_buffer.lookupTransform(fixed_frame, ps.getFrameID(), ros::Time(0));
        static_tfs_[ps.getNavFrameID()] = tf_buffer.lookupTransform(fixed_frame, ps.getNavFrameID(), ros::Time(0));
      }

      // get each POI
      YAML::Node pois_config = surface_config["pois"];
      for (size_t k = 0; k < pois_config.size(); k++)
      {
        // parse the name and frame
        YAML::Node poi_config = pois_config[k];
        PointOfInterest poi(poi_config["name"].as<string>(), poi_config["frame_id"].as<string>());
        // get our TF information
        static_tfs_[poi.getFrameID()] = tf_buffer.lookupTransform(fixed_frame, poi.getFrameID(), ros::Time(0));
        surface.addPointOfInterest(poi);
      }

      room.addSurface(surface);
    }

    // add it to the global world
    world_.addRoom(room);
  }

  drive_and_search_as_.start();
  drive_to_surface_as_.start();
  ROS_INFO("High Level Actions Successfully Initialized");
  okay_ = true;
#else
  ROS_ERROR("Unsupported version of YAML. Config files could not be parsed.");
  okay_ = false;
#endif
}

bool HighLevelActions::okay() const
{
  return okay_;
}

void HighLevelActions::driveAndSearch(const interactive_world_msgs::DriveAndSearchGoalConstPtr &goal)
{
  // ignore case
  string item = boost::to_upper_copy(goal->item);
  string surface_name = boost::to_upper_copy(goal->surface);
  ROS_INFO("Searching for '%s' on the '%s'...", item.c_str(), surface_name.c_str());

  interactive_world_msgs::DriveAndSearchFeedback feedback;
  interactive_world_msgs::DriveAndSearchResult result;
  // default to false
  result.success = false;

  // check each room for the surface
  vector<const Surface *> surfaces = world_.findSurfaces(surface_name);
  // check if any results were found
  if (surfaces.size() == 0)
  {
    drive_and_search_as_.setSucceeded(result, "Could not locate surface.");
    return;
  }

  // now check each surface
  for (size_t i = 0; i < surfaces.size(); i++)
  {
    const Surface *cur = surfaces[i];
    // go to each placement surface
    for (size_t j = 0; j < cur->getNumPlacementSurfaces(); j++)
    {
      const PlacementSurface &ps = cur->getPlacementSurface(j);

      // attempt to drive
      feedback.message = "Attempting to drive to " + ps.getNavFrameID();
      drive_and_search_as_.publishFeedback(feedback);
      if (!this->driveHelper(ps.getNavFrameID()))
      {
        ROS_WARN("Could not drive to %s, will continue the search elsewhere.", ps.getNavFrameID().c_str());
        continue;
      }

      // look at the surface frame
      feedback.message = "Attempting to look at " + ps.getFrameID();
      drive_and_search_as_.publishFeedback(feedback);
      carl_dynamixel::LookAtFrame look;
      look.request.frame = ps.getFrameID();
      if (!look_at_frame_srv_.call(look))
      {
        ROS_WARN("Could not look at %s, will continue the search elsewhere.", ps.getFrameID().c_str());
        continue;
      }

      // perform a segmentation request
      feedback.message = "Attempting to segment the surface";
      drive_and_search_as_.publishFeedback(feedback);
      std_srvs::Empty segment;
      segment_srv_.call(segment);
      if (!segment_srv_.call(segment))
      {
        ROS_WARN("Could not segment surface, will continue the search elsewhere.");
        continue;
      }
    }
  }
}

void HighLevelActions::driveToSurface(const interactive_world_msgs::DriveToSurfaceGoalConstPtr &goal)
{
  // ignore case
  string surface_name = boost::to_upper_copy(goal->surface);
  string poi_name = boost::to_upper_copy(goal->poi);
  ROS_INFO("Driving to '%s'...", surface_name.c_str());

  interactive_world_msgs::DriveToSurfaceFeedback feedback;
  interactive_world_msgs::DriveToSurfaceResult result;
  // default to false
  result.success = false;

  // search each room
  vector<const Surface *> surfaces = world_.findSurfaces(surface_name);
  // check if any results were found
  if (surfaces.size() == 0)
  {
    drive_to_surface_as_.setSucceeded(result, "Could not locate surface.");
    return;
  }

  // check if we can find the POI in any of the surfaces, if not, default to the first
  const Surface *surface = surfaces[0];
  const PointOfInterest *poi = NULL;
  if (poi_name.size() > 0)
  {
    for (size_t i = 0; i < surfaces.size() && poi == NULL; i++)
    {
      // search each POI
      const Surface *cur = surfaces[i];
      for (size_t j = 0; j < cur->getNumPointsOfInterest() && poi == NULL; j++)
      {
        // check the name
        if (boost::to_upper_copy(cur->getPointOfInterest(j).getName()) == poi_name)
        {
          poi = &(cur->getPointOfInterest(j));
          surface = surfaces[i];
        }
      }
    }
  }

  // now check which point is closest (use POI over point)
  double best_distance = numeric_limits<double>::infinity();
  size_t best_index = 0;
  if (poi != NULL)
  {
    tf2::Transform t_poi_world = this->tfFromTFMessage(static_tfs_[poi->getFrameID()].transform);
    // check each navigation point
    for (size_t i = 0; i < surface->getNumPlacementSurfaces(); i++)
    {
      // grab the TF for the placement surface
      const PlacementSurface &pc = surface->getPlacementSurface(i);
      tf2::Transform t_pc_world = this->tfFromTFMessage(static_tfs_[pc.getNavFrameID()].transform);
      // now check the distance
      double distance = t_pc_world.getOrigin().distance(t_poi_world.getOrigin());
      if (distance < best_distance)
      {
        best_distance = distance;
        best_index = i;
      }
    }
  } else
  {
    // pick the closest point
    for (size_t i = 0; i < surface->getNumPlacementSurfaces(); i++)
    {
      // grab the TF for the placement surface
      const PlacementSurface &pc = surface->getPlacementSurface(i);
      tf2::Transform t_pc_world = this->tfFromTFMessage(static_tfs_[pc.getNavFrameID()].transform);
      tf2::Vector3 point(goal->point.x, goal->point.y, goal->point.z);
      // now check the distance
      double distance = t_pc_world.getOrigin().distance(point);
      if (distance < best_distance)
      {
        best_distance = distance;
        best_index = i;
      }
    }
  }

  // now attempt to drive
  feedback.message = "Attempting to drive to " + surface->getPlacementSurface(best_index).getNavFrameID();
  drive_to_surface_as_.publishFeedback(feedback);
  if (this->driveHelper(surface->getPlacementSurface(best_index).getNavFrameID()))
  {
    result.success = true;
    drive_to_surface_as_.setSucceeded(result, "Success!");
  } else
  {
    drive_to_surface_as_.setSucceeded(result, "Could not navigate correctly.");
  }
}

bool HighLevelActions::driveHelper(const std::string &frame_id)
{
  // lock for the client
  boost::mutex::scoped_lock lock(nav_mutex_);

  move_base_msgs::MoveBaseGoal nav_goal;
  // drive to the nav frame
  nav_goal.target_pose.header.frame_id = frame_id;
  nav_goal.target_pose.pose.orientation.w = QUATERNION_90_ROTATE;
  nav_goal.target_pose.pose.orientation.z = QUATERNION_90_ROTATE;
  nav_ac_.sendGoal(nav_goal);
  bool completed = nav_ac_.waitForResult(ac_wait_time_);
  bool succeeded = (nav_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
  return completed && succeeded;
}

tf2::Transform HighLevelActions::tfFromTFMessage(const geometry_msgs::Transform &tf)
{
  // construct and return
  return tf2::Transform(tf2::Quaternion(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w),
      tf2::Vector3(tf.translation.x, tf.translation.y, tf.translation.z));
}
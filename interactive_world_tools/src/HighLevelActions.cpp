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
    : private_node_("~"), nav_ac_("/move_base", true), ac_wait_time_(AC_WAIT_TIME), fixed_frame_("map"),
      home_arm_ac_("/carl_moveit_wrapper/common_actions/arm_action", true),
      drive_and_search_as_(
          private_node_, "drive_and_search", boost::bind(&HighLevelActions::driveAndSearch, this, _1), false
      ),
      drive_to_surface_as_(
          private_node_, "drive_to_surface", boost::bind(&HighLevelActions::driveToSurface, this, _1), false
      )
{
  // set defaults
  recognized_objects_counter_ = 0;
  string world_file(ros::package::getPath("interactive_world_tools") + "/config/world.yaml");

  // grab any parameters we need
  private_node_.getParam("world_config", world_file);
  private_node_.getParam("fixed_frame", fixed_frame_);

  // setup services and topic
  look_at_frame_srv_ = node_.serviceClient<carl_dynamixel::LookAtFrame>("/asus_controller/look_at_frame");
  segment_srv_ = node_.serviceClient<std_srvs::Empty>("/rail_segmentation/segment");
  find_surface_srv_ = private_node_.advertiseService("find_surface", &HighLevelActions::findSurfaceCallback, this);
  get_surfaces_srv_ = private_node_.advertiseService("get_surfaces", &HighLevelActions::getSurfacesCallback, this);
  transform_to_surface_frame_srv_ = private_node_.advertiseService("transform_to_surface_frame",
                                                                   &HighLevelActions::transformToSurfaceFrame, this);
  recognized_objects_sub_ = node_.subscribe("/object_recognition_listener/recognized_objects", 1,
                                            &HighLevelActions::recognizedObjectsCallback, this);

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
    static_tfs_[room.getFrameID()] = tf_buffer.lookupTransform(fixed_frame_, room.getFrameID(), ros::Time(0));

    // get each surface
    YAML::Node surfaces_config = room_config["surfaces"];
    for (size_t j = 0; j < surfaces_config.size(); j++)
    {
      YAML::Node surface_config = surfaces_config[j];
      // parse the name, frame, and size
      Surface surface(surface_config["name"].as<string>(), surface_config["frame_id"].as<string>(),
                      surface_config["width"].as<double>(), surface_config["height"].as<double>());
      // get our TF information
      static_tfs_[surface.getFrameID()] = tf_buffer.lookupTransform(fixed_frame_, surface.getFrameID(), ros::Time(0));

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
        static_tfs_[ps.getFrameID()] = tf_buffer.lookupTransform(fixed_frame_, ps.getFrameID(), ros::Time(0));
        static_tfs_[ps.getNavFrameID()] = tf_buffer.lookupTransform(fixed_frame_, ps.getNavFrameID(), ros::Time(0));
      }

      // get each POI
      YAML::Node pois_config = surface_config["pois"];
      for (size_t k = 0; k < pois_config.size(); k++)
      {
        // parse the name and frame
        YAML::Node poi_config = pois_config[k];
        PointOfInterest poi(poi_config["name"].as<string>(), poi_config["frame_id"].as<string>());
        // get our TF information
        static_tfs_[poi.getFrameID()] = tf_buffer.lookupTransform(fixed_frame_, poi.getFrameID(), ros::Time(0));
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

bool HighLevelActions::getSurfacesCallback(interactive_world_msgs::GetSurfaces::Request & req,
                                           interactive_world_msgs::GetSurfaces::Response & resp)
{
  for (size_t i = 0; i < world_.getNumRooms(); i++)
  {
    const Room &room = world_.getRoom(i);
    for (size_t j = 0; j < room.getNumSurfaces(); j++)
    {
      resp.surfaces.push_back(room.getSurface(j).getName());
    }
  }

  return true;
}

bool HighLevelActions::transformToSurfaceFrame(interactive_world_msgs::TransformToSurfaceFrame::Request &req,
    interactive_world_msgs::TransformToSurfaceFrame::Response &resp)
{
  // check the fixed frame
  tf2::Transform t_pose_world;
  if (req.pose.header.frame_id != fixed_frame_)
  {
    ROS_WARN("Find surface request not in the global fixed frame. Will wait for TF information which can take time...");
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    // give the buffer time to fill
    sleep(tf_buffer.getCacheLength().sec / 10);
    geometry_msgs::TransformStamped tf = tf_buffer.lookupTransform(fixed_frame_, req.pose.header.frame_id,
                                                                   ros::Time(0));
    // transform the pose
    tf2::Transform t_req_world = this->tfFromTFMessage(tf.transform);
    tf2::Transform t_pose_req = this->tfFromPoseMessage(req.pose.pose);
    t_pose_world = t_req_world * t_pose_req;
  } else
  {
    t_pose_world = this->tfFromPoseMessage(req.pose.pose);
  }

  // check for surfaces (always use the first)
  vector<const Surface *> surfaces = world_.findSurfaces(req.surface);
  if (surfaces.empty())
  {
    ROS_WARN("Could not find surface %s.", req.surface.c_str());
    return false;
  } else
  {
    const Surface *surface = surfaces[0];
    // get the pose in relation to the surface
    tf2::Transform t_surface_world = this->tfFromTFMessage(static_tfs_[surface->getFrameID()].transform);
    tf2::Transform t_pose_surface = t_surface_world.inverseTimes(t_pose_world);

    resp.pose.header.frame_id = surface->getFrameID();
    resp.pose.pose.position.x = t_pose_surface.getOrigin().x();
    resp.pose.pose.position.y = t_pose_surface.getOrigin().y();
    resp.pose.pose.position.z = t_pose_surface.getOrigin().z();
    resp.pose.pose.orientation.x = t_pose_surface.getRotation().x();
    resp.pose.pose.orientation.y = t_pose_surface.getRotation().y();
    resp.pose.pose.orientation.z = t_pose_surface.getRotation().z();
    resp.pose.pose.orientation.w = t_pose_surface.getRotation().w();

    return true;
  }
}

bool HighLevelActions::findSurfaceCallback(interactive_world_msgs::FindSurface::Request &req,
    interactive_world_msgs::FindSurface::Response &resp)
{
  // check the fixed frame
  tf2::Transform t_pose_world;
  if (req.pose.header.frame_id != fixed_frame_)
  {
    ROS_WARN("Find surface request not in the global fixed frame. Will wait for TF information which can take time...");
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    // give the buffer time to fill
    sleep(tf_buffer.getCacheLength().sec / 10);
    geometry_msgs::TransformStamped tf = tf_buffer.lookupTransform(fixed_frame_, req.pose.header.frame_id,
                                                                   ros::Time(0));
    // transform the pose
    tf2::Transform t_req_world = this->tfFromTFMessage(tf.transform);
    tf2::Transform t_pose_req = this->tfFromPoseMessage(req.pose.pose);
    t_pose_world = t_req_world * t_pose_req;
  } else
  {
    t_pose_world = this->tfFromPoseMessage(req.pose.pose);
  }

  // go through each surface
  for (size_t i = 0; i < world_.getNumRooms(); i++)
  {
    const Room &room = world_.getRoom(i);
    for (size_t j = 0; j < room.getNumSurfaces(); j++)
    {
      const Surface &surface = room.getSurface(j);

      // get the pose in relation to the surface
      tf2::Transform t_surface_world = this->tfFromTFMessage(static_tfs_[surface.getFrameID()].transform);
      tf2::Transform t_pose_surface = t_surface_world.inverseTimes(t_pose_world);

      // first check if we are inside that surface's bounding box
      if ((surface.getWidth() / 2.0 >= abs(t_pose_surface.getOrigin().x()))
          && (surface.getHeight() / 2.0 >= abs(t_pose_surface.getOrigin().y())))
      {
        // check each placement surface
        double closest = numeric_limits<double>::infinity();
        for (size_t k = 0; k < surface.getNumPlacementSurfaces(); k++)
        {
          const PlacementSurface &ps = surface.getPlacementSurface(k);

          // distance to surface
          tf2::Transform t_ps_world = this->tfFromTFMessage(static_tfs_[ps.getFrameID()].transform);
          double distance = t_ps_world.getOrigin().distance(t_surface_world.getOrigin());
          if (distance < closest)
          {
            // check if we are within the height of a placement surface
//            if (abs(static_tfs_[ps.getFrameID()].transform.translation.z - t_pose_world.getOrigin().z())
//                <= SURFACE_Z_THRESH)
//            {
              closest = distance;
              resp.placement_surface_frame_id = ps.getFrameID();
//            }
          }
        }

        if (closest < numeric_limits<double>::infinity())
        {
          // found the surface
          resp.surface = surface.getName();
          resp.surface_frame_id = surface.getFrameID();
          return true;
        }
        // if we are here, we were in the bounding box but no on the surface
        ROS_WARN("Point was found to be inside of '%s' but not on the surface itself.", surface.getName().c_str());
        return true;
      }
    }
  }

  // if we are here, the point was not on a surface
  ROS_WARN("Point was not found to be on any surface.");
  return true;
}

void HighLevelActions::driveAndSearch(const interactive_world_msgs::DriveAndSearchGoalConstPtr &goal)
{
  // ignore case
  string item_name = boost::to_upper_copy(goal->item);
  string surface_name = boost::to_upper_copy(goal->surface);
  ROS_INFO("Searching for '%s' on the '%s'...", item_name.c_str(), surface_name.c_str());

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

      // retract the arm if we are going to be searching
      rail_manipulation_msgs::ArmGoal arm_goal;
      arm_goal.action = rail_manipulation_msgs::ArmGoal::RETRACT;
      home_arm_ac_.sendGoal(arm_goal);
      bool completed = home_arm_ac_.waitForResult(ac_wait_time_);
      bool succeeded = (home_arm_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
      bool success = home_arm_ac_.getResult()->success;
      if (!completed || !succeeded || !success)
      {
        ROS_ERROR("Could not retract arm during search.");
        drive_and_search_as_.setSucceeded(result, "Could not retract arm during search.");
      }

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

      // check how many recognized objects we have so far (a segment request will first send an empty list)
      uint32_t counter;
      {
        boost::mutex::scoped_lock lock(recognized_objects_mutex_);
        counter = recognized_objects_counter_;
      }

      // perform a segmentation request
      feedback.message = "Attempting to segment the surface.";
      drive_and_search_as_.publishFeedback(feedback);
      std_srvs::Empty segment;
      if (!segment_srv_.call(segment))
      {
        ROS_WARN("Could not segment surface, will continue the search elsewhere.");
        continue;
      }

      // spin and wait
      feedback.message = "Waiting for recognition results.";
      drive_and_search_as_.publishFeedback(feedback);
      bool finished = false;
      while (!finished)
      {
        {
          boost::mutex::scoped_lock lock(recognized_objects_mutex_);
          finished = recognized_objects_counter_ == (counter + 2);
        }
      }

      // parse the recognition results
      boost::mutex::scoped_lock lock(recognized_objects_mutex_);
      for (size_t i = 0; i < recognized_objects_.objects.size(); i++)
      {
        if (recognized_objects_.objects[i].recognized)
        {
          string cur_name = boost::to_upper_copy(recognized_objects_.objects[i].name);
          // check for a name match
          if (cur_name == item_name)
          {
            // found at least one match
            result.success = true;
            result.objects.objects.push_back(recognized_objects_.objects[i]);
          }
        }
      }

      // check if we found something in this zone
      if (result.success)
      {
        drive_and_search_as_.setSucceeded(result, "Object located.");
        return;
      }
    }
  }

  drive_and_search_as_.setSucceeded(result, "Could not locate object.");
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

tf2::Transform HighLevelActions::tfFromPoseMessage(const geometry_msgs::Pose &pose)
{
  // construct and return
  return tf2::Transform(tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                        tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

void HighLevelActions::recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &objects)
{
  // lock for the list
  boost::mutex::scoped_lock lock(recognized_objects_mutex_);

  recognized_objects_ = objects;
  recognized_objects_counter_++;
}
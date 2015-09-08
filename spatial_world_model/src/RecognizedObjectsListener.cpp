#include <spatial_world_model/RecognizedObjectsListener.h>

#include <interactive_world_msgs/StoreObservation.h>
#include <interactive_world_msgs/FindSurface.h>
#include <interactive_world_msgs/FindObservations.h>
#include <interactive_world_msgs/SetObservationsRemoved.h>
#include <interactive_world_msgs/TransformToSurfaceFrame.h>

using namespace std;
using namespace rail::interactive_world;

RecognizedObjectsListener::RecognizedObjectsListener()
{
  incoming_cleared_objects_ = false;
  store_observation_srv_ = node_.serviceClient<interactive_world_msgs::StoreObservation>(
      "/spatial_world_model_server/store_observation"
  );
  find_surface_srv_ = node_.serviceClient<interactive_world_msgs::FindSurface>(
      "/high_level_action_server/find_surface"
  );
  transform_to_surface_frame_srv_ = node_.serviceClient<interactive_world_msgs::TransformToSurfaceFrame>(
      "/high_level_action_server/transform_to_surface_frame"
  );
  find_observations_srv_ = node_.serviceClient<interactive_world_msgs::FindObservations>(
      "/spatial_world_model_server/find_observations"
  );
  recognized_objects_sub_ = node_.subscribe("/object_recognition_listener/recognized_objects", 1,
                                            &RecognizedObjectsListener::recognizedObjectsCallback, this);
  set_observations_removed_srv_ = node_.serviceClient<interactive_world_msgs::SetObservationsRemoved>(
      "/spatial_world_model_server/set_observations_removed"
  );
  ROS_INFO("Recognized Objects Listener Initialized");
}

void RecognizedObjectsListener::recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &objects)
{
  // ignore cleared objects
  incoming_cleared_objects_ = !incoming_cleared_objects_;
  if (objects.objects.empty() && incoming_cleared_objects_)
  {
    return;
  }

  ROS_INFO("Attmpting to store %ld objects in the spatial world model...", objects.objects.size());

  // list of objects seen
  vector<string> seen_objects;
  // list of placement surfaces found
  map<string, vector<string> > seen_surfaces;

  // go through each recognized object
  uint32_t count = 0;
  for (size_t i = 0; i < objects.objects.size(); i++)
  {
    const rail_manipulation_msgs::SegmentedObject &object = objects.objects[i];
    if (object.recognized)
    {
      // determine the surface
      interactive_world_msgs::FindSurface surface;
      surface.request.pose.header.frame_id = object.point_cloud.header.frame_id;
      surface.request.pose.pose.position.x = object.centroid.x;
      surface.request.pose.pose.position.y = object.centroid.y;
      surface.request.pose.pose.position.z = object.centroid.z - (object.height / 2.0);
      surface.request.pose.pose.orientation = object.orientation;

      if (find_surface_srv_.call(surface))
      {
        // check if the surface was found
        if (!surface.response.surface.empty())
        {
          seen_objects.push_back(object.name);

          // fill the request
          interactive_world_msgs::StoreObservation store;
          store.request.surface = surface.response.surface;
          store.request.surface_frame_id = surface.response.surface_frame_id;
          store.request.placement_surface_frame_id = surface.response.placement_surface_frame_id;
          store.request.item = object.name;
          store.request.item_conf = object.confidence;

          // store the surface
          seen_surfaces[surface.response.surface_frame_id].push_back(surface.response.placement_surface_frame_id);

          // transform the position to the surface frame
          interactive_world_msgs::TransformToSurfaceFrame transform_to_surface_frame;
          transform_to_surface_frame.request.pose = surface.request.pose;
          transform_to_surface_frame.request.surface = surface.response.surface;
          if (transform_to_surface_frame_srv_.call(transform_to_surface_frame))
          {
            store.request.x = transform_to_surface_frame.response.pose.pose.position.x;
            store.request.y = transform_to_surface_frame.response.pose.pose.position.y;
            store.request.z = transform_to_surface_frame.response.pose.pose.position.z;
            tf2::Quaternion q(transform_to_surface_frame.response.pose.pose.orientation.x,
                              transform_to_surface_frame.response.pose.pose.orientation.y,
                              transform_to_surface_frame.response.pose.pose.orientation.z,
                              transform_to_surface_frame.response.pose.pose.orientation.w);
            store.request.theta = q.getAngle();

            // attempt to store the result
            if (store_observation_srv_.call(store))
            {
              count++;
            } else
            {
              ROS_WARN("Failed to store current observation of object '%s'.", object.name.c_str());
            }
          } else
          {
            ROS_WARN("Failed to transform pose to surface frame for '%s'.", object.name.c_str());
          }
        } else
        {
          ROS_WARN("Failed to find surface for object '%s'.", object.name.c_str());
        }
      } else
      {
        ROS_WARN("Failed to call surface finder.");
      }
    }
  }

  ROS_INFO("Stored %d item observations.", count);

  // now remove things that are gone
  interactive_world_msgs::SetObservationsRemoved remove;
  for (map<string, vector<string> >::iterator iter = seen_surfaces.begin(); iter != seen_surfaces.end(); ++iter)
  {
    // unique placement surfaces
    vector<string> &placement_surfaces = iter->second;
    sort(placement_surfaces.begin(), placement_surfaces.end());
    vector<string>::iterator last = unique(iter->second.begin(), iter->second.end());
    placement_surfaces.erase(last, placement_surfaces.end());
    for (size_t i = 0; i < placement_surfaces.size(); i++)
    {
      // grab anything that should have been on the table
      interactive_world_msgs::FindObservations find;
      find.request.surface_frame_id = iter->first;
      find.request.placement_surface_frame_id = placement_surfaces[i];
      find.request.removed = false;
      find_observations_srv_.call(find);
      // go through and remove objects we did not see
      for (size_t j = 0; j < find.response.items.size(); j++)
      {
        if (std::find(seen_objects.begin(), seen_objects.end(), find.response.items[j]) == seen_objects.end())
        {
          remove.request.ids.push_back(find.response.ids[j]);
        }
      }
    }
  }

  if (!remove.request.ids.empty())
  {
    ROS_INFO("Setting %ld old observations as removed.", remove.request.ids.size());
    set_observations_removed_srv_.call(remove);
  }
}
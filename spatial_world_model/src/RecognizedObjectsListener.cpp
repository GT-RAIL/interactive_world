#include <spatial_world_model/RecognizedObjectsListener.h>

#include <interactive_world_msgs/StoreObservation.h>
#include <interactive_world_msgs/FindSurface.h>
#include <interactive_world_msgs/TransformToSurfaceFrame.h>

using namespace std;
using namespace rail::interactive_world;

RecognizedObjectsListener::RecognizedObjectsListener()
{
  store_observation_srv_ = node_.serviceClient<interactive_world_msgs::StoreObservation>(
      "/spatial_world_model_server/store_observation"
  );
  find_surface_srv_ = node_.serviceClient<interactive_world_msgs::FindSurface>(
      "/high_level_action_server/find_surface"
  );
  transform_to_surface_frame_srv_ = node_.serviceClient<interactive_world_msgs::TransformToSurfaceFrame>(
      "/high_level_action_server/transform_to_surface_frame"
  );
  recognized_objects_sub_ = node_.subscribe("/object_recognition_listener/recognized_objects", 1,
      &RecognizedObjectsListener::recognizedObjectsCallback, this);
  ROS_INFO("Recognized Objects Listener Initialized");
}

void RecognizedObjectsListener::recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &objects)
{
  if (objects.objects.empty())
  {
    return;
  }

  ROS_INFO("Attmpting to store %ld objects in the spatial world model...", objects.objects.size());

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
          // fill the request
          interactive_world_msgs::StoreObservation store;
          store.request.surface = surface.response.surface; // TODO
          store.request.placement_surface_frame_id = surface.response.frame_id; // TODO
          store.request.item = object.name;
          store.request.item_conf = object.confidence;

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
}
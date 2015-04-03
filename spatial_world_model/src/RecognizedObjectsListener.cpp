#include <spatial_world_model/RecognizedObjectsListener.h>

using namespace std;
using namespace rail::interactive_world;

RecognizedObjectsListener::RecognizedObjectsListener()
{
  store_observation_srv_ = node_.serviceClient<interactive_world_msgs::StoreObservation>(
      "/spatial_world_model_server/store_observation"
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

  // go through each recognized object
  uint32_t count = 0;
  for (size_t i = 0; i < objects.objects.size(); i++)
  {
    const rail_manipulation_msgs::SegmentedObject &object = objects.objects[i];
    if (object.recognized)
    {
      // TODO determine the surface

      // fill the request
      interactive_world_msgs::StoreObservation store;
      store.request.surface = "TODO"; // TODO
      store.request.placement_surface_frame_id = "TODO"; // TODO
      store.request.item = object.name;
      store.request.item_conf = object.confidence;
      // TODO what frame are these?
      store.request.x = object.centroid.x;
      store.request.y = object.centroid.y;
      store.request.z = object.centroid.z;
      // TODO get orientation
      store.request.theta = 0;

      // attempt to store the result
      if (store_observation_srv_.call(store))
      {
        count++;
      } else
      {
        ROS_WARN("Failed to store current observation of object '%s'.", object.name.c_str());
      }
    }
  }

  ROS_INFO("Stored %d item observations.", count);
}
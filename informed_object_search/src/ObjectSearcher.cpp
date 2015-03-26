/*!
 * \file ObjectSearcher.cpp
 * \brief The main object search node class.
 *
 * The object searcher node is responsible for searching for a given object given information obtained from the
 * interactive world project. Associated ROS information is maintained in this class.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#include <boost/algorithm/string.hpp>
#include <informed_object_search/ObjectSearcher.h>
#include <interactive_world_msgs/LoadModels.h>
#include <map>

using namespace std;
using namespace rail::interactive_world;

ObjectSearcher::ObjectSearcher()
    : private_node_("~"), search_ac_("/high_level_action_server/drive_and_search", true), ac_wait_time_(AC_WAIT_TIME),
      as_(private_node_, "object_search", boost::bind(&ObjectSearcher::objectSearch, this, _1), false)
{
  // setup any services/topics we need
  model_fetcher_srv_ = node_.serviceClient<interactive_world_msgs::LoadModels>(
      "/interactive_world_model_fetcher/load_models"
  );

  // start the action server
  as_.start();

  // wait for the action server
  search_ac_.waitForServer(ac_wait_time_);

  ROS_INFO("Object Searcher Successfully Initialized");
}


void ObjectSearcher::objectSearch(const interactive_world_msgs::ObjectSearchGoalConstPtr &goal)
{
  // ignore case
  string item = boost::to_upper_copy(goal->item);
  ROS_INFO("Searching for '%s'...", item.c_str());

  interactive_world_msgs::ObjectSearchFeedback feedback;
  interactive_world_msgs::ObjectSearchResult result;
  // default to false
  result.success = false;

  // request models
  feedback.message = "Requesting trainied models...";
  as_.publishFeedback(feedback);

  // grab each model from the database
  vector<interactive_world_msgs::Model> models;
  for (size_t i = 0; i < goal->models.size(); i++)
  {
    // create and send a request
    interactive_world_msgs::LoadModels loader;
    loader.request.condition_id = goal->models[i];
    if (model_fetcher_srv_.call(loader))
    {
      // search for the object we are looking for
      vector<interactive_world_msgs::Model> &cur = loader.response.models.models;
      for (size_t j = 0; j < cur.size(); j++)
      {
        // check if the names match and restrict search to surfaces
        string placement_item = boost::to_upper_copy(cur[j].placement.item.name);
        string placement_reference = boost::to_upper_copy(cur[j].placement.reference_frame_id);
        if (cur[j].placement.surface.name.size() > 0 && (placement_item == item || placement_reference == item))
        {
          models.push_back(cur[j]);
        }
      }
    } else
    {
      ROS_WARN("Could not load model %i.", goal->models[i]);
      as_.setSucceeded(result, "Could not load all models from the RMS database.");
      return;
    }
  }

  // tally up confidences for each surface
  map<string, double> confidences;
  for (size_t i = 0; i < models.size(); i++)
  {
    // get the current surface name
    const interactive_world_msgs::Model &m = models[i];
    string surface_name = m.placement.surface.name;
    if (confidences.find(surface_name) == confidences.end())
    {
      confidences[surface_name] = 0.0;
    }

    // add to the confidence
    confidences[surface_name] += m.decision_value;
  }

  while (!confidences.empty())
  {
    // check for the current best
    double lowest = numeric_limits<double>::infinity();
    string best_surface;
    for (map<string, double>::iterator iter = confidences.begin(); iter != confidences.end(); ++iter)
    {
      string key = iter->first;
      double value = iter->second;
      if (value < lowest)
      {
        best_surface = key;
        lowest = value;
      }
    }

    // search for the highest confidence
    feedback.message = "Searching " + best_surface;
    as_.publishFeedback(feedback);

    // perform a higher level search request
    interactive_world_msgs::DriveAndSearchGoal search_goal;
    search_goal.item = item;
    search_goal.surface = best_surface;
    search_ac_.sendGoal(search_goal);
    bool completed = search_ac_.waitForResult(ac_wait_time_);
    bool succeeded = (search_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
    bool success = search_ac_.getResult()->success;
    if (!completed || !succeeded || !success)
    {
      feedback.message = "Object not found on " + search_goal.surface;
      as_.publishFeedback(feedback);
    } else
    {
      // TODO object found
    }

    confidences.erase(best_surface);
  }

  // success
  as_.setSucceeded(result, "Object search failed.");
}

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
        string check = boost::to_upper_copy(cur[j].placement.item.name);
        if (check == item && cur[j].placement.surface.name.size() > 0)
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

  // search for the highest confidence
  feedback.message = "Starting with highest confidence model...";
  as_.publishFeedback(feedback);

  while (!models.empty())
  {
    // check for the current best
    double highest = -numeric_limits<double>::infinity();
    size_t best = 0;
    for (size_t i = 0; i < models.size(); i++)
    {
      if (models[i].decision_value > highest)
      {
        best = i;
        highest = models[i].decision_value;
      }
    }

    cout << "SEARCHING " << models[best].placement.surface.name << endl;
    interactive_world_msgs::DriveAndSearchGoal search_goal;
    // drive to the nav frame
    search_goal.item = item;
    search_goal.surface = models[best].placement.surface.name;
    // TODO do not rotate
    search_ac_.sendGoal(search_goal);
    bool completed = search_ac_.waitForResult(ac_wait_time_);
    bool succeeded = (search_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
    if (!completed || !succeeded)
    {
      as_.setSucceeded(result, "Could not search.");
      cout << "no, no no no." << endl;
      return;
    } else {
      //TODO
      cout << "SWEET" << endl;
//      result.success = true;
//      as_.setSucceeded(result, "Success!");
//      return;
    }

    models.erase(models.begin() + best);
  }

  // success
  result.success = true;
  as_.setSucceeded(result, "Success!");
}

#ifndef INTERACTIVE_WORLD_MODEL_FETCHER_HPP_
#define INTERACTIVE_WORLD_MODEL_FETCHER_HPP_

#include <ros/ros.h>
#include <interactive_world_msgs/LoadModels.h>
#include <geometry_msgs/Pose.h>
#include <jsoncpp/json/json.h>

#define DEFAULT_RMS_HOST "localhost"

class interactive_world_model_fetcher
{
public:
  interactive_world_model_fetcher();

  bool load_models_cb(interactive_world_msgs::LoadModels::Request &req, interactive_world_msgs::LoadModels::Response &resp);

private:
  geometry_msgs::Pose parse_pose(Json::Value &json);

  geometry_msgs::Point parse_point(Json::Value &json);

  interactive_world_msgs::Surface parse_surface(Json::Value &json);

  interactive_world_msgs::PointOfInterest parse_poi(Json::Value &json);

  ros::NodeHandle private_;

  std::string host_;
  ros::ServiceServer load_models_;
};

static size_t curl_write_cb(void *contents, size_t size, size_t nmemb, void *userp);

int main(int argc, char **argv);

#endif

#ifndef INTERACTIVE_WORLD_PARSER_HPP_
#define INTERACTIVE_WORLD_PARSER_HPP_

#include <ros/ros.h>
#include <rms/rms.hpp>
#include <jsoncpp/json/json.h>
#include <geometry_msgs/Pose.h>
#include <interactive_world_msgs/Configuration.h>
#include <interactive_world_msgs/TaskTrainingData.h>
#include <interactive_world_msgs/LearnModels.h>
#include <interactive_world_msgs/Parse.h>
#include <tf2/LinearMath/Transform.h>
#include <map>

#define Z_AXIS_TF2 tf2::Vector3(0, 0, 1)

class interactive_world_parser
{
public:
  interactive_world_parser();

  ~interactive_world_parser();

  void parse(int study_id);

  void learn();

  void save();

  void store();

  bool parse_and_save_cb(interactive_world_msgs::Parse::Request &req, interactive_world_msgs::Parse::Response &resp);

  bool parse_and_store_cb(interactive_world_msgs::Parse::Request &req, interactive_world_msgs::Parse::Response &resp);

private:
  interactive_world_msgs::Configuration parse_json_config(Json::Value &config);

  geometry_msgs::Pose parse_json_pose(Json::Value &position, double rotation);

  tf2::Transform parse_json_tf(Json::Value &position, double rotation);

  tf2::Transform pose_to_tf(geometry_msgs::Pose &pose);

  void parse_json_placement(Json::Value &placement, interactive_world_msgs::Configuration &config, unsigned int condition_id, std::vector<librms::log> logs);

  void add_placement_to_data(uint condition_id, tf2::Transform &tf, interactive_world_msgs::Item item, interactive_world_msgs::Room room, interactive_world_msgs::Surface surface, std::string reference_frame_id);

  ros::NodeHandle private_;

  librms::rms *client_;
  std::string host_, user_, password_, database_;
  int port_;
  std::map<uint, interactive_world_msgs::TaskTrainingData> data_;
  ros::ServiceServer parse_and_store_, parse_and_save_;
  ros::ServiceClient learn_hypotheses_;
};

int main(int argc, char **argv);

#endif

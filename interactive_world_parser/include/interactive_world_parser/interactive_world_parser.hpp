#ifndef INTERACTIVE_WORLD_PARSER_HPP_
#define INTERACTIVE_WORLD_PARSER_HPP_

#include <ros/ros.h>
#include <rms/rms.hpp>
#include <jsoncpp/json/json.h>
#include <geometry_msgs/Pose.h>
#include <interactive_world_msgs/Configuration.h>
#include <tf2/LinearMath/Transform.h>
#include <map>
#include <vector>

#define DEFAULT_STUDY_ID 1

#define Z_AXIS_TF2 tf2::Vector3(0, 0, 1)

class interactive_world_parser
{
public:
  interactive_world_parser();

  ~interactive_world_parser();

  void parse();

private:
  interactive_world_msgs::Configuration parse_json_config(Json::Value &config);

  geometry_msgs::Pose parse_json_pose(Json::Value &position, double rotation);

  tf2::Transform parse_json_tf(Json::Value &position, double rotation);

  tf2::Transform pose_to_tf(geometry_msgs::Pose &pose);

  void parse_json_placement(Json::Value &placement, interactive_world_msgs::Configuration &config, unsigned int condition_id, std::vector<librms::log> logs);

  ros::NodeHandle node_, private_;

  librms::rms *client_;
  std::string host_, user_, password_, database_;
  int port_, study_id_;
  std::map<uint, std::map<std::string, std::vector<tf2::Transform> > > data_;
};

int main(int argc, char **argv);

#endif

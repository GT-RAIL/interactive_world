#include <interactive_world_parser/interactive_world_parser.hpp>
#include <fstream>
#include <pwd.h>

using namespace std;

interactive_world_parser::interactive_world_parser() :
    private_("~")
{
  // grab params
  private_.param("host", host_, string(RMS_DEFAULT_SERVER));
  private_.param("port", port_, RMS_DEFAULT_PORT);
  private_.param("user", user_, string(RMS_DEFAULT_USER));
  private_.param("password", password_, string(RMS_DEFAULT_PASSWORD));
  private_.param("database", database_, string(RMS_DEFAULT_DATABASE));
  private_.param("study_id", study_id_, DEFAULT_STUDY_ID);

  // create the client
  client_ = new librms::rms(host_, (unsigned int) port_, user_, password_, database_);

  // attempt to connection
  if (!client_->connect())
  {
    ROS_ERROR("Fatal: could not connect to the RMS database.");
    exit(EXIT_FAILURE);
  }

  // setup servers
  parse_and_store_ = private_.advertiseService("parse_and_store", &interactive_world_parser::parse_and_store_cb, this);
  parse_and_save_ = private_.advertiseService("parse_and_save", &interactive_world_parser::parse_and_save_cb, this);

  // wait for external service
  learn_hypotheses_ = private_.serviceClient<interactive_world_msgs::LearnModels>("/interactive_world_learner/learn_hypotheses");
  ROS_INFO("Waiting for /interactive_world_learner/learn_hypotheses...");
  learn_hypotheses_.waitForExistence(ros::DURATION_MAX);

  ROS_INFO("Interactive World Parser Initialized");
}


interactive_world_parser::~interactive_world_parser()
{
  delete client_;
}

static int __limit__ = 0;

static bool cond(const interactive_world_msgs::PlacementSet &set)
{
  return set.placements.size() < __limit__;
}


void interactive_world_parser::parse()
{
  ROS_INFO("=== BEGINING PARSE ===");
  data_.clear();
  librms::study study = client_->get_study((unsigned int) study_id_);
  ROS_INFO("Running over study \"%s\"...", study.get_name().c_str());

  // grab the conditions
  vector<librms::condition> &conditions = study.get_conditions();
  ROS_INFO("+ Found %d conditions.", (int) conditions.size());
  for (uint i = 0; i < conditions.size(); i++)
  {
    librms::condition &condition = conditions[i];
    ROS_INFO("+ Parsing condition \"%s\"...", condition.get_name().c_str());
    int num_experiments = 0;
    int num_completed = 0;
    int num_placements = 0;

    vector<librms::slot> &slots = condition.get_slots();
    for (uint j = 0; j < slots.size(); j++)
    {
      ROS_INFO("++ Parsing slot %d/%d...", j + 1, (int) slots.size());
      librms::slot &slot = slots[j];
      if (slot.has_appointment())
      {
        // check for logs
        librms::appointment &appointment = slot.get_appointment();
        vector<librms::log> &logs = appointment.get_logs();
        if (logs.size() > 1)
        {
          bool completion = false;
          num_experiments++;
          interactive_world_msgs::Configuration config;
          for (uint k = 0; k < logs.size(); k++)
          {
            librms::log &log = logs[k];

            // parse the JSON
            Json::Value json;
            Json::Reader reader;
            reader.parse(log.get_entry(), json, false);

            // grab the initial configuration
            if (log.get_label().compare("config") == 0)
            {
              config = parse_json_config(json);
            } else if (!completion && log.get_label().compare("completion") == 0)
            {
              completion = true;
              num_completed++;
            } else
            {
              // parse the placements
              num_placements++;
              parse_json_placement(json, config, condition.get_id(), logs);
            }
          }
        }
      }
    }
    ROS_INFO("++ %d/%d completed sessions with %d placements.", num_completed, num_experiments, num_placements);
    __limit__ = (int) (num_placements * 0.05);
    ROS_INFO("++ Removing data sets with less than %d placements...", __limit__);
    vector<interactive_world_msgs::PlacementSet> &v = data_[condition.get_id()].data;
    v.erase(remove_if(v.begin(), v.end(), cond), v.end());
  }

  ROS_INFO("=== PARSE FINISHED ===");
}

void interactive_world_parser::save()
{
  ROS_INFO("Writing data...");
  // save the data
  for (map<uint, interactive_world_msgs::TaskTrainingData>::iterator it = data_.begin(); it != data_.end(); ++it)
  {
    uint condition_id = it->first;
    interactive_world_msgs::TaskTrainingData &training = it->second;
    for (uint i = 0; i < training.data.size(); i++)
    {
      interactive_world_msgs::PlacementSet &placements = training.data[i];
      stringstream ss;
      ss << (getpwuid(getuid()))->pw_dir << "/" << condition_id << "-" << placements.item.name << "-in-" << placements.room.name << "-on-" << placements.surface.name << "-wrt-" << placements.reference_frame_id << ".csv";
      ofstream file;
      file.open(ss.str().c_str());
      for (uint j = 0; j < placements.placements.size(); j++)
      {
        interactive_world_msgs::Placement &placement = placements.placements[j];
        file << placement.position.x << "," << placement.position.y << "," << placement.position.z << "," << placement.rotation << endl;
      }
      file.close();
    }
  }
  ROS_INFO("Finished!");
}

void interactive_world_parser::learn()
{
  ROS_INFO("=== BEGINING MODEL LEARNING ===");
  // save the data
  for (map<uint, interactive_world_msgs::TaskTrainingData>::iterator it = data_.begin(); it != data_.end(); ++it)
  {
    uint condition_id = it->first;
    interactive_world_msgs::TaskTrainingData &training = it->second;
    ROS_INFO("Sending condition %d data off to jinteractiveworld...", condition_id);
    // construct the message
    interactive_world_msgs::LearnModels srv;
    srv.request.host = host_;
    srv.request.port = port_;
    srv.request.user = user_;
    srv.request.password = password_;
    srv.request.database = database_;
    srv.request.condition_id = condition_id;
    srv.request.data = it->second;
    learn_hypotheses_.call(srv);
    ROS_INFO("Condition %d models learned.", condition_id);
  }
  ROS_INFO("=== MODEL LEARNING FINISHED ===");
}

bool interactive_world_parser::parse_and_store_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  this->parse();
  this->learn();
  return true;
}


bool interactive_world_parser::parse_and_save_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
{
  this->parse();
  this->save();
  return true;
}


interactive_world_msgs::Configuration interactive_world_parser::parse_json_config(Json::Value &config)
{
  interactive_world_msgs::Configuration configuration;

  // task description
  configuration.task = config["task"].asString();
  // objects
  Json::Value objects = config["objects"];
  for (uint i = 0; i < objects.size(); i++)
  {
    Json::Value cur_object = objects[i];
    interactive_world_msgs::Item item;
    item.name = cur_object["name"].asString();
    item.width = cur_object["width"].asDouble();
    item.height = cur_object["height"].asDouble();
    configuration.items.push_back(item);
  }
  // rooms
  Json::Value rooms = config["rooms"];
  for (uint i = 0; i < rooms.size(); i++)
  {
    Json::Value cur_room = rooms[i];
    interactive_world_msgs::Room room;
    room.name = cur_room["name"].asString();
    room.width = cur_room["width"].asDouble();
    room.height = cur_room["height"].asDouble();
    room.pose = parse_json_pose(cur_room["position"], cur_room["rotation"].asDouble());
    // surfaces
    Json::Value surfaces = cur_room["furniture"];
    for (uint j = 0; j < surfaces.size(); j++)
    {
      Json::Value cur_surface = surfaces[j];
      interactive_world_msgs::Surface surface;
      surface.name = cur_surface["name"].asString();
      surface.width = cur_surface["width"].asDouble();
      surface.height = cur_surface["height"].asDouble();
      surface.pose = parse_json_pose(cur_surface["position"], cur_surface["rotation"].asDouble());
      // POIs
      Json::Value pois = cur_surface["poi"];
      for (uint k = 0; k < pois.size(); k++)
      {
        Json::Value cur_poi = pois[k];
        interactive_world_msgs::PointOfInterest poi;
        poi.name = cur_poi["name"].asString();
        poi.width = cur_poi["width"].asDouble();
        poi.height = cur_poi["height"].asDouble();
        poi.pose = parse_json_pose(cur_poi["position"], cur_poi["rotation"].asDouble());
        surface.pois.push_back(poi);
      }
      room.surfaces.push_back(surface);
    }
    configuration.rooms.push_back(room);
  }

  return configuration;
}

void interactive_world_parser::parse_json_placement(Json::Value &placement, interactive_world_msgs::Configuration &config, unsigned int condition_id, vector<librms::log> logs)
{
  // parse the JSON
  Json::Value json_furniture = placement["furniture"];
  Json::Value json_furniture_position = json_furniture["position"];
  Json::Value json_furniture_surface = json_furniture["surface"];
  Json::Value json_furniture_surface_position = json_furniture_surface["position"];
  Json::Value json_object = json_furniture_surface["object"];
  Json::Value json_object_position = json_object["position"];

  string room_name = placement["name"].asString();
  string surface_name = json_furniture["name"].asString();
  string object_name = json_object["name"].asString();

  interactive_world_msgs::Item item_msg;
  interactive_world_msgs::Room room_msg;
  interactive_world_msgs::Surface surface_msg;

  // get what we need from the config
  vector<interactive_world_msgs::PointOfInterest> pois;
  for (uint i = 0; i < config.rooms.size(); i++)
  {
    interactive_world_msgs::Room &room = config.rooms[i];
    if (room.name.compare(room_name) == 0)
    {
      room_msg = config.rooms[i];
      for (uint j = 0; j < room.surfaces.size(); j++)
      {
        interactive_world_msgs::Surface &surface = room.surfaces[j];
        if (surface.name.compare(surface_name) == 0)
        {
          surface_msg = room.surfaces[j];
          pois = surface.pois;
          break;
        }
      }
      break;
    }
  }
  for (uint i = 0; i < config.items.size(); i++)
  {
    interactive_world_msgs::Item &item = config.items[i];
    if (item.name.compare(object_name) == 0)
    {
      item_msg = config.items[i];
      break;
    }
  }

  // grab the surface info
  tf2::Transform t_room_surface = parse_json_tf(json_furniture_position, json_furniture["rotation"].asDouble());

  // placement surface
  tf2::Transform t_surface_psurface = parse_json_tf(json_furniture_surface_position, json_furniture_surface["rotation"].asDouble());

  // psurface to the object
  tf2::Transform t_psurface_object = parse_json_tf(json_object_position, json_object["rotation"].asDouble());

  // get the object in room and furniture coords
  tf2::Transform t_surface_object = t_surface_psurface * t_psurface_object;
  tf2::Transform t_room_object = t_room_surface * t_surface_psurface * t_psurface_object;

  // store the placement on the surface
  add_placement_to_data(condition_id, t_surface_object, item_msg, room_msg, surface_msg, surface_name);

  // check for POIs
  vector<string> used_names;
  for (uint i = 0; i < pois.size(); i++)
  {
    // check if this is a new POI type
    string cur_poi_name = pois[i].name;
    if (find(used_names.begin(), used_names.end(), cur_poi_name) == used_names.end())
    {
      used_names.push_back(cur_poi_name);

      // find the closest of this kind
      double closest_dist = numeric_limits<double>::infinity();
      tf2::Transform closest_t;
      for (uint j = 0; j < pois.size(); j++)
      {
        interactive_world_msgs::PointOfInterest cur_poi = pois[j];
        if (cur_poi_name.compare(cur_poi.name) == 0)
        {
          double distance = t_surface_object.getOrigin().distance(tf2::Vector3(cur_poi.pose.position.x, cur_poi.pose.position.y, cur_poi.pose.position.z));
          if (distance < closest_dist)
          {
            closest_dist = distance;
            closest_t = pose_to_tf(cur_poi.pose);
          }
        }
      }
      // convert to POI frame
      tf2::Transform t_poi_object = closest_t.inverseTimes(t_surface_object);
      // store the value
      add_placement_to_data(condition_id, t_poi_object, item_msg, room_msg, surface_msg, cur_poi_name);
    }
  }

  // place next to each closest item
  for (int i = 0; i < config.items.size(); i++)
  {
    interactive_world_msgs::Item &item = config.items[i];
    if (item.name.compare(object_name) != 0)
    {
      // search the logs again for the object
      double closest_dist = numeric_limits<double>::infinity();
      tf2::Transform closest_t;
      for (int j = 0; j < logs.size(); j++)
      {
        librms::log &log = logs[j];
        // grab the initial configuration
        if (log.get_label().compare("config") != 0 && log.get_label().compare("completion") != 0)
        {
          // parse the JSON
          Json::Value json;
          Json::Reader reader;
          reader.parse(log.get_entry(), json, false);
          // parse the JSON
          Json::Value json_furniture2 = json["furniture"];
          Json::Value json_furniture_position2 = json_furniture2["position"];
          Json::Value json_furniture_surface2 = json_furniture2["surface"];
          Json::Value json_furniture_surface_position2 = json_furniture_surface2["position"];
          Json::Value json_object2 = json_furniture_surface2["object"];
          Json::Value json_object_position2 = json_object2["position"];
          // only compare on the same furniture
          if (json["name"].asString().compare(room_name) == 0 && json_furniture2["name"].asString().compare(surface_name) == 0 && json_object2["name"].asString().compare(item.name) == 0)
          {
            // convert to the surface frame
            tf2::Transform t_surface_psurface2 = parse_json_tf(json_furniture_surface_position2, json_furniture_surface2["rotation"].asDouble());
            tf2::Transform t_psurface_object2 = parse_json_tf(json_object_position2, json_object2["rotation"].asDouble());
            tf2::Transform t_surface_object2 = t_surface_psurface2 * t_psurface_object2;
            // compute the distance
            double distance = t_surface_object.getOrigin().distance(t_surface_object2.getOrigin());
            if (distance < closest_dist)
            {
              closest_dist = distance;
              closest_t = t_surface_object2;
            }
          }
        }
      }
      // check if we found a closest match
      if (closest_dist < numeric_limits<double>::infinity())
      {
        // convert to object2 frame
        tf2::Transform t_object2_object = closest_t.inverseTimes(t_surface_object);
        // store the value
        add_placement_to_data(condition_id, t_object2_object, item_msg, room_msg, surface_msg, item.name);
      }
    }
  }
}

tf2::Transform interactive_world_parser::parse_json_tf(Json::Value &position, double rotation)
{
  tf2::Quaternion q(Z_AXIS_TF2, rotation);
  tf2::Vector3 v(position["x"].asDouble(), position["y"].asDouble(), position["z"].asDouble());
  tf2::Transform tf(q, v);
  return tf;
}

tf2::Transform interactive_world_parser::pose_to_tf(geometry_msgs::Pose &pose)
{
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Vector3 v(pose.position.x, pose.position.y, pose.position.z);
  tf2::Transform tf(q, v);
  return tf;
}

geometry_msgs::Pose interactive_world_parser::parse_json_pose(Json::Value &position, double rotation)
{
  geometry_msgs::Pose p;
  p.position.x = position["x"].asDouble();
  p.position.y = position["y"].asDouble();
  p.position.z = position["z"].asDouble();

  // create the rotation
  tf2::Quaternion q(Z_AXIS_TF2, rotation);
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();

  return p;
}

void interactive_world_parser::add_placement_to_data(uint condition_id, tf2::Transform &tf, interactive_world_msgs::Item item, interactive_world_msgs::Room room, interactive_world_msgs::Surface surface, string reference_frame_id)
{
  if (data_.find(condition_id) == data_.end())
  {
    data_[condition_id] = interactive_world_msgs::TaskTrainingData();
  }

  interactive_world_msgs::TaskTrainingData &training = data_[condition_id];

  // create the placement
  interactive_world_msgs::Placement placement;
  placement.item = item;
  placement.room = room;
  placement.surface = surface;
  placement.reference_frame_id = reference_frame_id;
  placement.rotation = tf.getRotation().getAngle();
  placement.position.x = tf.getOrigin().x();
  placement.position.y = tf.getOrigin().y();
  placement.position.z = tf.getOrigin().z();

  // search for the correct frame
  for (uint i = 0; i < training.data.size(); i++)
  {
    interactive_world_msgs::PlacementSet &placements = training.data[i];
    if (placements.item.name.compare(item.name) == 0 && placements.room.name.compare(room.name) == 0 && placements.surface.name.compare(surface.name) == 0 && placements.reference_frame_id.compare(reference_frame_id) == 0)
    {
      placements.placements.push_back(placement);
      return;
    }
  }

  // new entry
  interactive_world_msgs::PlacementSet set;
  set.reference_frame_id = reference_frame_id;
  set.item = item;
  set.room = room;
  set.surface = surface;
  set.placements.push_back(placement);
  training.data.push_back(set);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "interactive_world_parser");

  interactive_world_parser parser;

  ros::spin();
}

#include <spatial_world_model/spatial_world_model_server.h>

using namespace std;

spatial_world_model_server::spatial_world_model_server() : private_("~")
{
  // grab params
  private_.param("host", host_, string(SWM_DEFAULT_SERVER));
  private_.param("port", port_, SWM_DEFAULT_PORT);
  private_.param("user", user_, string(SWM_DEFAULT_USER));
  private_.param("password", password_, string(SWM_DEFAULT_PASSWORD));
  private_.param("database", database_, string(SWM_DEFAULT_DATABASE));

  // setup the client
  connected_ = false;
  conn_ = mysql_init(NULL);
  if (!mysql_real_connect(conn_, host_.c_str(), user_.c_str(), password_.c_str(), database_.c_str(), port_, NULL, 0))
  {
    ROS_ERROR("MySQL Error: %s", mysql_error(conn_));
    exit(EXIT_FAILURE);
  } else
  {
    connected_ = true;
  }

  // create the table if we need to
  mysql_free_result(this->query("DROP TABLE swms;"));
  MYSQL_RES *res = this->query(SWM_CREATE_TABLE);
  if (res)
  {
    mysql_free_result(res);
  }

  // setup ROS stuff
  store_observation_ = private_.advertiseService("store_observation", &spatial_world_model_server::store_observation_cb, this);

  ROS_INFO("Spatial World Model Initialized");
}

spatial_world_model_server::~spatial_world_model_server()
{
  if (connected_)
  {
    mysql_close(conn_);
  }
}

bool spatial_world_model_server::store_observation_cb(interactive_world_msgs::StoreObservation::Request &req, interactive_world_msgs::StoreObservation::Response &resp)
{
  // convert into an SQL statement
  stringstream ss;
  ss << "INSERT INTO `" << SWM_TABLE << "`"
      << "(`room`, `room_frame_id`, `room_conf`, `surface`, `surface_frame_id`, `surface_conf`, `item`, `item_frame_id`, `item_conf`, `x`, `y`, `z`, `theta`)"
      << "VALUES ("
      << "\"" << req.room << "\", "
      << "\"" << req.room_frame_id << "\", "
      << "" << req.room_conf << ", "
      << "\"" << req.surface << "\", "
      << "\"" << req.surface_frame_id << "\", "
      << "" << req.surface_conf << ", "
      << "\"" << req.item << "\", "
      << "\"" << req.item_frame_id << "\", "
      << "" << req.item_conf << ", "
      << "" << req.x << ", "
      << "" << req.y << ", "
      << "" << req.z << ", "
      << "" << req.theta
      << ");";

  MYSQL_RES *res = this->query(ss.str());
  if (res)
  {
    mysql_free_result(res);
  }

  return true;
}

bool spatial_world_model_server::connected()
{
  return connected_;
}

MYSQL_RES *spatial_world_model_server::query(string query)
{
  if (mysql_query(conn_, query.c_str()) == 0)
  {
    // parse and get it
    return mysql_use_result(conn_);
  } else
  {
    ROS_ERROR("MySQL Error: %s", mysql_error(conn_));
  }

  // something went wrong
  return NULL;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "spatial_world_model_server");
  spatial_world_model_server server;
  ros::spin();
}

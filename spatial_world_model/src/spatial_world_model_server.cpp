#include <spatial_world_model/spatial_world_model_server.h>

#include <boost/algorithm/string.hpp>

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
  MYSQL_RES *res = this->query(SWM_CREATE_TABLE);
  if (res)
  {
    mysql_free_result(res);
  }

  // setup ROS stuff
  store_observation_ = private_.advertiseService("store_observation", &spatial_world_model_server::store_observation_cb, this);
  find_observations_ = private_.advertiseService("find_observations",
      &spatial_world_model_server::find_observations_cb, this);

  ROS_INFO("Spatial World Model Initialized");
}

spatial_world_model_server::~spatial_world_model_server()
{
  if (connected_)
  {
    mysql_close(conn_);
  }
}

bool spatial_world_model_server::find_observations_cb(interactive_world_msgs::FindObservations::Request & req,
    interactive_world_msgs::FindObservations::Response & resp)
{
  // convert into an SQL statement
  stringstream ss;
  ss << "SELECT * FROM `" << SWM_TABLE << "` WHERE UPPER(item)=\"" << boost::to_upper_copy(req.item) << "\" "
      << "ORDER BY `time` DESC;";
  string query = ss.str();

  MYSQL_RES *res = this->query(query);
  if (res)
  {
    MYSQL_ROW row;
    while ((row = mysql_fetch_row(res)) != NULL)
    {
      resp.surfaces.push_back(string(row[1]));
      resp.placement_surface_frame_ids.push_back(string(row[2]));
      resp.item_confs.push_back(atof(row[4]));
      resp.xs.push_back(atof(row[5]));
      resp.ys.push_back(atof(row[6]));
      resp.zs.push_back(atof(row[7]));
      resp.thetas.push_back(atof(row[8]));;
      resp.times.push_back(ros::Time(this->extract_time(row[9]), 0));
    }
    mysql_free_result(res);
  }

  if (resp.surfaces.empty())
  {
    ROS_WARN("No instances of %s found in the spatial world model.", req.item.c_str());
    return false;
  } else
  {
    return true;
  }
}

bool spatial_world_model_server::store_observation_cb(interactive_world_msgs::StoreObservation::Request &req, interactive_world_msgs::StoreObservation::Response &resp)
{
  // convert into an SQL statement
  stringstream ss;
  ss << "INSERT INTO `" << SWM_TABLE << "`"
      << "(`surface`, `placement_surface_frame_id`, `item`, `item_conf`, `x`, `y`, `z`, `theta`)"
      << "VALUES ("
      << "\"" << req.surface << "\", "
      << "\"" << req.placement_surface_frame_id << "\", "
      << "\"" << req.item << "\", "
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

time_t spatial_world_model_server::extract_time(const string &str) const
{
  // set values we don't need to be 0
  struct tm t;
  bzero(&t, sizeof(t));
  sscanf(str.c_str(), "%d-%d-%d %d:%d:%d", &t.tm_year, &t.tm_mon, &t.tm_mday, &t.tm_hour, &t.tm_min, &t.tm_sec);
  // correct the information for C time
  t.tm_year -= 1900;
  t.tm_mon -= 1;
  // eastern time
  t.tm_hour -= 5;
  return mktime(&t);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "spatial_world_model_server");
  spatial_world_model_server server;
  ros::spin();
}

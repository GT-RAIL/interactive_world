#include <interactive_world_tools/interactive_world_model_fetcher.hpp>
#include <curl/curl.h>

using namespace std;

interactive_world_model_fetcher::interactive_world_model_fetcher() : private_("~")
{
  // grab params
  private_.param("host", host_, string(DEFAULT_RMS_HOST));

  load_models_ = private_.advertiseService("load_models", &interactive_world_model_fetcher::load_models_cb, this);

  ROS_INFO("Interactive World Model Fetcher Initialized");
}

bool interactive_world_model_fetcher::load_models_cb(interactive_world_msgs::LoadModels::Request &req, interactive_world_msgs::LoadModels::Response &resp)
{
  string buffer;
  bool success = true;

  // make a request via cURL
  CURL *curl = curl_easy_init();
  // create the URL
  stringstream ss;
  ss << "http://" << host_ << "/iwmodels/view/" << req.condition_id;
  curl_easy_setopt(curl, CURLOPT_URL, ss.str().c_str());
  // follow redirects
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  // set up the read
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_cb);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer);

  // perform the request
  CURLcode result = curl_easy_perform(curl);
  if (result != CURLE_OK)
  {
    ROS_ERROR("Interactive World Model Fetch Failed: %s", curl_easy_strerror(result));
    success = false;
  } else
  {
    // check if we had a valid response
    long code;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
    if (code != 200)
    {
      ROS_ERROR("Interactive World Model Fetch Failed (Return Code %i)", (int) code);
      success = false;
    } else
    {
      // parse the JSON to get the to other JSON value
      Json::Value json1;
      Json::Reader reader1;
      reader1.parse(buffer, json1, false);
      Json::Value entity = json1["Iwmodel"];
      string value = entity["value"].asString();

      Json::Value json2;
      Json::Reader reader2;
      reader2.parse(value, json2, false);

      // convert to a message
      Json::Value models = json2["models"];
      for (int i = 0; i < models.size(); i++)
      {
        // parse the model
        Json::Value cur_model = models[i];
        interactive_world_msgs::Model cur_model_msg;
        cur_model_msg.decision_value = cur_model["decision_value"].asDouble();
        cur_model_msg.sigma_x = cur_model["sigma_x"].asDouble();
        cur_model_msg.sigma_y = cur_model["sigma_y"].asDouble();
        cur_model_msg.sigma_z = cur_model["sigma_z"].asDouble();
        cur_model_msg.sigma_theta = cur_model["sigma_theta"].asDouble();

        // parse the placement
        Json::Value placement = cur_model["placement"];
        cur_model_msg.placement.rotation = placement["rotation"].asDouble();
        cur_model_msg.placement.reference_frame_id = placement["reference_frame_id"].asString();
        cur_model_msg.placement.position = parse_point(placement["position"]);

        // parse the item
        Json::Value item = placement["item"];
        cur_model_msg.placement.item.name = item["name"].asString();
        cur_model_msg.placement.item.width = item["width"].asDouble();
        cur_model_msg.placement.item.height = item["height"].asDouble();

        // parse the room
        Json::Value room = placement["room"];
        cur_model_msg.placement.room.name = room["name"].asString();
        cur_model_msg.placement.room.width = room["width"].asDouble();
        cur_model_msg.placement.room.height = room["height"].asDouble();
        cur_model_msg.placement.room.pose = parse_pose(room["pose"]);

        // parse the room surfaces
        Json::Value room_surfaces = room["surfaces"];
        for (int j = 0; j < room_surfaces.size(); j++)
        {
          cur_model_msg.placement.room.surfaces.push_back(parse_surface(room_surfaces[j]));
        }

        // parse the placement surface
        cur_model_msg.placement.surface = parse_surface(placement["surface"]);

        resp.models.models.push_back(cur_model_msg);
      }
    }
  }

  curl_easy_cleanup(curl);
  return success;
}

interactive_world_msgs::Surface interactive_world_model_fetcher::parse_surface(Json::Value &json)
{
  interactive_world_msgs::Surface surface;
  surface.name = json["name"].asString();
  surface.width = json["width"].asDouble();
  surface.height = json["height"].asDouble();
  surface.pose = parse_pose(json["pose"]);

  // parse POIs

  Json::Value pois = json["pois"];
  for (int i = 0; i < pois.size(); i++)
  {
    surface.pois.push_back(parse_poi(pois[i]));
  }
  return surface;
}


interactive_world_msgs::PointOfInterest interactive_world_model_fetcher::parse_poi(Json::Value &json)
{
  interactive_world_msgs::PointOfInterest poi;
  poi.name = json["name"].asString();
  poi.width = json["width"].asDouble();
  poi.height = json["height"].asDouble();
  poi.pose = parse_pose(json["pose"]);
  return poi;
}

geometry_msgs::Pose interactive_world_model_fetcher::parse_pose(Json::Value &json)
{
  geometry_msgs::Pose pose;
  pose.position = parse_point(json["position"]);
  Json::Value orientation = json["orientation"];
  pose.orientation.x = orientation["x"].asDouble();
  pose.orientation.y = orientation["y"].asDouble();
  pose.orientation.z = orientation["z"].asDouble();
  pose.orientation.w = orientation["w"].asDouble();
  return pose;
}

geometry_msgs::Point interactive_world_model_fetcher::parse_point(Json::Value &json)
{
  geometry_msgs::Point point;
  point.x = json["x"].asDouble();
  point.y = json["y"].asDouble();
  point.z = json["z"].asDouble();
  return point;
}

static size_t curl_write_cb(void *contents, size_t size, size_t nmemb, void *userp)
{
  // store the result in the buffer
  ((string *) userp)->append((char *) contents, size * nmemb);
  return size * nmemb;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "interactive_world_model_fetcher");
  interactive_world_model_fetcher fetcher;
  ros::spin();
}

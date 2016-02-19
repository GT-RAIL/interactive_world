#include "informed_object_search/ObjectOnSurfaceModel.h"
#include <ros/time.h>

using namespace std;
using namespace rail::interactive_world;

ObjectOnSurfaceModel::ObjectOnSurfaceModel(const string &object, const string &surface)
    : object_(object), surface_(surface)
{
  dirty_ = false;
  last_seen_ = 0;
  mu_ = 0;
}

const string &ObjectOnSurfaceModel::getObject() const
{
  return object_;
}

void ObjectOnSurfaceModel::setObject(const std::string &object)
{
  object_ = object;
}

const string &ObjectOnSurfaceModel::getSurface() const
{
  return surface_;
}

void ObjectOnSurfaceModel::setSurface(const std::string &surface)
{
  surface_ = surface;
}

time_t ObjectOnSurfaceModel::getLastSeen() const
{
  return last_seen_;
}

void ObjectOnSurfaceModel::addObservation(const time_t seen, const time_t removed)
{
  if (seen > last_seen_)
  {
    last_seen_ = seen;
  }
  struct observation cur;
  cur.seen = seen;
  cur.removed = removed;
  observations_.push_back(cur);
  dirty_ = true;
}

double ObjectOnSurfaceModel::getMu()
{
  if (dirty_)
  {
    // average
    mu_ = 0;
    ros::Time now = ros::Time::now();
    for (size_t i = 0; i < observations_.size(); i++)
    {
      // check if the observation is still current
      double duration;
      if (observations_[i].removed == 0)
      {
        duration = now.toSec() - observations_[i].seen;
      } else
      {
        duration = observations_[i].removed - observations_[i].seen;
      }
      // put into minutes
      duration /= 60.0;
      mu_ += duration;
    }
    mu_ /= (double) observations_.size();
    dirty_ = false;
  }

  return mu_;
}

double ObjectOnSurfaceModel::getLambda()
{
  return 1.0 / this->getMu();
}

double ObjectOnSurfaceModel::getCurrentProbability()
{
  ros::Time now = ros::Time::now();
  // x in minutes
  double x = (now.toSec() - this->getLastSeen()) / 60.0;
  double lambda = this->getLambda();
  return exp(-lambda * x);
}
/*!
 * \file PointOfInterest.cpp
 * \brief Point of interest configuration information.
 *
 * A POI contains a name and associated frame.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#include <interactive_world_tools/PointOfInterest.h>

using namespace std;
using namespace rail::interactive_world;

PointOfInterest::PointOfInterest(const string &name, const string &frame_id) : name_(name), frame_id_(frame_id)
{
}

const string &PointOfInterest::getName() const
{
  return name_;
}

void PointOfInterest::setName(const string &name)
{
  name_ = name;
}

const string &PointOfInterest::getFrameID() const
{
  return frame_id_;
}

void PointOfInterest::setFrameID(const string &frame_id)
{
  frame_id_ = frame_id;
}

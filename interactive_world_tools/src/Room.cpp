/*!
 * \file Room.cpp
 * \brief Room configuration information.
 *
 * A room consists of a series of surfaces.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#include <boost/algorithm/string.hpp>
#include <interactive_world_tools/Room.h>
#include <stdexcept>

using namespace std;
using namespace rail::interactive_world;

Room::Room(const string &name, const string &frame_id) : name_(name), frame_id_(frame_id)
{
}

const string &Room::getName() const
{
  return name_;
}

void Room::setName(const string &name)
{
  name_ = name;
}

const string &Room::getFrameID() const
{
  return frame_id_;
}

void Room::setFrameID(const string &frame_id)
{
  frame_id_ = frame_id;
}

const vector<Surface> &Room::getSurfaces() const
{
  return surfaces_;
}

size_t Room::getNumSurfaces() const
{
  return surfaces_.size();
}

const Surface &Room::getSurface(const size_t index) const
{
  // check the index value first
  if (index < surfaces_.size())
  {
    return surfaces_[index];
  } else
  {
    throw std::out_of_range("Room::getSurface : Surface index does not exist.");
  }
}

void Room::addSurface(const Surface &room)
{
  surfaces_.push_back(room);
}

void Room::removeSurface(const size_t index)
{
  // check the index value first
  if (index < surfaces_.size())
  {
    surfaces_.erase(surfaces_.begin() + index);
  } else
  {
    throw std::out_of_range("Room::removeSurface : Surface index does not exist.");
  }
}

const Surface &Room::findSurface(const string &name) const
{
  string name_uc = boost::to_upper_copy(name);
  for (size_t i = 0; i < surfaces_.size(); i++)
  {
    // check the direct name
    if (boost::to_upper_copy(surfaces_[i].getName()) == name_uc)
    {
      return surfaces_[i];
    }

    // check each alias
    for (size_t j = 0; j < surfaces_[i].getNumAliases(); j++)
    {
      if (boost::to_upper_copy(surfaces_[i].getAlias(j)) == name_uc)
      {
        return surfaces_[i];
      }
    }
  }
  throw std::out_of_range("Room::findSurface : Surface name does not exist.");
}

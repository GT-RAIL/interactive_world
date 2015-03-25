/*!
 * \file World.cpp
 * \brief World configuration information.
 *
 * A world consists of a series of rooms and surfaces. Surfaces can have points of interest as well.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#include <interactive_world_tools/World.h>
#include <stdexcept>
#include <ros/ros.h>

using namespace std;
using namespace rail::interactive_world;

World::World()
{
}

const vector<Room> &World::getRooms() const
{
  return rooms_;
}

size_t World::getNumRooms() const
{
  return rooms_.size();
}

const Room &World::getRoom(const size_t index) const
{
  // check the index value first
  if (index < rooms_.size())
  {
    return rooms_[index];
  } else
  {
    throw std::out_of_range("World::getRoom : Room index does not exist.");
  }
}

void World::addRoom(const Room &room)
{
  rooms_.push_back(room);
}

void World::removeRoom(const size_t index)
{
  // check the index value first
  if (index < rooms_.size())
  {
    rooms_.erase(rooms_.begin() + index);
  } else
  {
    throw std::out_of_range("World::removeRoom : Room index does not exist.");
  }
}

vector<const Surface *> World::findSurfaces(const string &name)
{
  vector<const Surface *> surfaces;
  // go through each room
  for (size_t i = 0; i < rooms_.size(); i++)
  {
    try
    {
      // check the current room
      const Surface &s = rooms_[i].findSurface(name);
      surfaces.push_back(&s);
    } catch (std::out_of_range &e)
    {
      // ignore, try the next room
    }
  }

  return surfaces;
}
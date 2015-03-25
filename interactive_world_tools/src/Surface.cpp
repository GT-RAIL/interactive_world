/*!
 * \file Surface.cpp
 * \brief Surface configuration information.
 *
 * A surface consists of a name with associated placement frames and points of interest.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#include <interactive_world_tools/Surface.h>
#include <stdexcept>

using namespace std;
using namespace rail::interactive_world;

Surface::Surface(const string &name, const string &frame_id, const double width, const double height)
    : name_(name), frame_id_(frame_id)
{
  width_ = width;
  height_ = height;
}

const string &Surface::getName() const
{
  return name_;
}

void Surface::setName(const string &name)
{
  name_ = name;
}

const string &Surface::getFrameID() const
{
  return frame_id_;
}

void Surface::setFrameID(const string &frame_id)
{
  frame_id_ = frame_id;
}

double Surface::getWidth() const
{
  return width_;
}

void Surface::setWidth(const double width)
{
  width_ = width;
}

double Surface::getHeight() const
{
  return height_;
}


void Surface::setHeight(const double height)
{
  height_ = height;
}

const vector<PlacementSurface> &Surface::getPlacementSurfaces() const
{
  return placement_surfaces_;
}

size_t Surface::getNumPlacementSurfaces() const
{
  return placement_surfaces_.size();
}

const PlacementSurface &Surface::getPlacementSurface(const size_t index) const
{
  // check the index value first
  if (index < placement_surfaces_.size())
  {
    return placement_surfaces_[index];
  } else
  {
    throw std::out_of_range("Surface::getPlacementSurface : Placement surface index does not exist.");
  }
}

void Surface::addPlacementSurface(const PlacementSurface &placement_surface)
{
  placement_surfaces_.push_back(placement_surface);
}

void Surface::removePlacementSurface(const size_t index)
{
  // check the index value first
  if (index < placement_surfaces_.size())
  {
    placement_surfaces_.erase(placement_surfaces_.begin() + index);
  } else
  {
    throw std::out_of_range("Surface::removePlacementSurface : Placement surface index does not exist.");
  }
}

const std::vector<PointOfInterest> &Surface::getPointsOfInterest() const
{
  return pois_;
}

size_t Surface::getNumPointsOfInterest() const
{
  return pois_.size();
}

const PointOfInterest &Surface::getPointOfInterest(const size_t index) const
{
  // check the index value first
  if (index < pois_.size())
  {
    return pois_[index];
  } else
  {
    throw std::out_of_range("Surface::getPointOfInterest : Point of interest index does not exist.");
  }
}

void Surface::addPointOfInterest(const PointOfInterest &point_of_interest)
{
  pois_.push_back(point_of_interest);
}

void Surface::removePointOfInterest(const size_t index)
{
  // check the index value first
  if (index < pois_.size())
  {
    pois_.erase(pois_.begin() + index);
  } else
  {
    throw std::out_of_range("Surface::removePointOfInterest : Point of interest index does not exist.");
  }
}

const std::vector<std::string> &Surface::getAliases() const
{
  return aliases_;
}

size_t Surface::getNumAliases() const
{
  return aliases_.size();
}

const std::string &Surface::getAlias(const size_t index) const
{
  // check the index value first
  if (index < aliases_.size())
  {
    return aliases_[index];
  } else
  {
    throw std::out_of_range("Surface::getAlias : Alias index does not exist.");
  }
}

void Surface::addAlias(const std::string &alias)
{
  aliases_.push_back(alias);
}

void Surface::removeAlias(const size_t index)
{
  // check the index value first
  if (index < aliases_.size())
  {
    aliases_.erase(aliases_.begin() + index);
  } else
  {
    throw std::out_of_range("Surface::removeAlias : Alias index does not exist.");
  }
}

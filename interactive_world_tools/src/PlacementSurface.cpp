/*!
 * \file PlacementSurface.cpp
 * \brief Point of interest configuration information.
 *
 * A POI contains a name and associated frame.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#include <interactive_world_tools/PlacementSurface.h>

using namespace std;
using namespace rail::interactive_world;

PlacementSurface::PlacementSurface(const string &frame_id, const string &nav_frame_id)
    : frame_id_(frame_id), nav_frame_id_(nav_frame_id)
{
}

const string &PlacementSurface::getFrameID() const
{
  return frame_id_;
}

void PlacementSurface::setFrameID(const string &frame_id)
{
  frame_id_ = frame_id;
}

const string &PlacementSurface::getNavFrameID() const
{
  return nav_frame_id_;
}

void PlacementSurface::setNavFrameID(const string &nav_frame_id)
{
  nav_frame_id_ = nav_frame_id;
}

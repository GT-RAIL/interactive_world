/*!
 * \file PlacementSurface.h
 * \brief Placement surface configuration information.
 *
 * A placement surface contains a frame ID and associated navigation frame ID.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#ifndef RAIL_INTERACTIVE_WORLD_PLACEMENT_SURFACE_H_
#define RAIL_INTERACTIVE_WORLD_PLACEMENT_SURFACE_H_

#include <string>

namespace rail
{
namespace interactive_world
{

/*!
 * \class PlacementSurface
 * \brief Placement surface configuration information.
 *
 * A placement surface contains a frame ID and associated navigation frame ID.
 */
class PlacementSurface
{
public:
  /*!
   * \brief Create a new PlacementSurface.
   *
   * Create a new PlacementSurface with the given frame IDs.
   *
   * \param frame_id The frame ID of the POI (defaults to the empty string).
   * \param nav_frame_id The name of the navigation frame ID (defaults to the empty string).
   */
  PlacementSurface(const std::string &frame_id = "", const std::string &nav_frame_id = "");

  /*!
   * \brief Frame ID value accessor.
   *
   * Get the frame ID value of this PlacementSurface.
   *
   * \return The frame ID value.
   */
  const std::string &getFrameID() const;

  /*!
   * \brief Frame ID value mutator.
   *
   * Set the frame ID value of this PlacementSurface.
   *
   * \param frame_id The new frame ID name value.
   */
  void setFrameID(const std::string &frame_id);

  /*!
   * \brief Navigation frame ID value accessor.
   *
   * Get the navigation frame ID value of this PlacementSurface.
   *
   * \return The navigation frame ID value.
   */
  const std::string &getNavFrameID() const;

  /*!
   * \brief Navigation frame ID value mutator.
   *
   * Set the navigation frame ID value of this PlacementSurface.
   *
   * \param frame_id The new navigation frame ID value.
   */
  void setNavFrameID(const std::string &nav_frame_id);

private:
  /*! Name of the frame ID and navigation frame ID. */
  std::string frame_id_, nav_frame_id_;
};

}
}

#endif

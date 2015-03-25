/*!
 * \file Surface.h
 * \brief Surface configuration information.
 *
 * A surface consists of a name with associated placement frames and points of interest.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#ifndef RAIL_INTERACTIVE_WORLD_SURFACE_H_
#define RAIL_INTERACTIVE_WORLD_SURFACE_H_

#include <interactive_world_tools/PlacementSurface.h>
#include <interactive_world_tools/PointOfInterest.h>
#include <string>
#include <vector>

namespace rail
{
namespace interactive_world
{

/*!
 * \class PlacementSurface
 * \brief Surface configuration information.
 *
 * A surface consists of a name and size with associated placement frames and points of interest.
 */
class Surface
{
public:
  /*!
   * \brief Create a new Surface.
   *
   * Create a new Surface with the given name and frame ID. POI and placement surface lists are empty.
   *
   * \param name The name of the surface (defaults to the empty string).
   * \param frame_id The frame ID of the surface (defaults to the empty string).
   * \param width The width of the surface along the X axis (defaults to 0).
   * \param height The height of the surface along the Y axis (defaults to 0).
   */
  Surface(const std::string &name = "", const std::string &frame_id = "", const double width = 0,
      const double height = 0);

  /*!
   * \brief Name value accessor.
   *
   * Get the name value of this Surface.
   *
   * \return The name value.
   */
  const std::string &getName() const;

  /*!
   * \brief Name value mutator.
   *
   * Set the name value of this Surface.
   *
   * \param name The new name value.
   */
  void setName(const std::string &name);

  /*!
   * \brief Frame ID value accessor.
   *
   * Get the frame ID value of this Surface.
   *
   * \return The frame ID value.
   */
  const std::string &getFrameID() const;

  /*!
   * \brief Frame ID value mutator.
   *
   * Set the frame ID value of this Surface.
   *
   * \param frame_id The new frame ID value.
   */
  void setFrameID(const std::string &frame_id);

  /*!
   * \brief Width value accessor.
   *
   * Get the width value of this Surface.
   *
   * \return The width value.
   */
  double getWidth() const;

  /*!
   * \brief Width value mutator.
   *
   * Set the width value of this Surface.
   *
   * \param width The new width value.
   */
  void setWidth(const double width);

  /*!
   * \brief Height value accessor.
   *
   * Get the height value of this Surface.
   *
   * \return The height value.
   */
  double getHeight() const;

  /*!
   * \brief Height value mutator.
   *
   * Set the height value of this Surface.
   *
   * \param height The new height value.
   */
  void setHeight(const double height);

  /*!
   * \brief Placement surfaces value accessor.
   *
   * Get the placement surfaces of this Surface.
   *
   * \return The placement surfaces.
   */
  const std::vector<PlacementSurface> &getPlacementSurfaces() const;

  /*!
   * \brief Placement surfaces size accessor.
   *
   * Get the number of placement surfaces of this Surface.
   *
   * \return The number of placement surfaces of this Surface.
   */
  size_t getNumPlacementSurfaces() const;

  /*!
   * \brief Placement surface value accessor.
   *
   * Get the placement surface of this Surface at the given index.
   *
   * \param i The index of the PlacementSurface to get.
   * \return The placement surface at the given index.
   * \throws std::out_of_range Thrown if the placement surface at the given index does not exist.
   */
  const PlacementSurface &getPlacementSurface(const size_t index) const;

  /*!
   * \brief Placement surface adder.
   *
   * Add the placement surface to this Surface.
   *
   * \param placement_surface The new placement surface to add.
   */
  void addPlacementSurface(const PlacementSurface &placement_surface);

  /*!
   * \brief Placement surface remover.
   *
   * Remove the placement surface at the given index. An invalid index results in no effect.
   *
   * \param i The index of the placement surface to remove.
   * \throws std::out_of_range Thrown if the placement surface at the given index does not exist.
   */
  void removePlacementSurface(const size_t index);

  /*!
   * \brief Aliases value accessor.
   *
   * Get the aliases of this Surface.
   *
   * \return The aliases of this surfaces.
   */
  const std::vector<std::string> &getAliases() const;

  /*!
   * \brief Aliases size accessor.
   *
   * Get the number of aliases of this Surface.
   *
   * \return The number of aliases of this Surface.
   */
  size_t getNumAliases() const;

  /*!
   * \brief Alias value accessor.
   *
   * Get the alias of this Surface at the given index.
   *
   * \param i The index of the alias to get.
   * \return The alias at the given index.
   * \throws std::out_of_range Thrown if the alias at the given index does not exist.
   */
  const std::string &getAlias(const size_t index) const;

  /*!
   * \brief Alias adder.
   *
   * Add the alias to this Surface.
   *
   * \param alias The new alias to add.
   */
  void addAlias(const std::string &alias);

  /*!
   * \brief Alias remover.
   *
   * Remove the alias at the given index. An invalid index results in no effect.
   *
   * \param i The index of the alias to remove.
   * \throws std::out_of_range Thrown if the alias at the given index does not exist.
   */
  void removeAlias(const size_t index);

  /*!
   * \brief Points of interest value accessor.
   *
   * Get the points of interest of this Surface.
   *
   * \return The points of interest.
   */
  const std::vector<PointOfInterest> &getPointsOfInterest() const;

  /*!
   * \brief Point of interests size accessor.
   *
   * Get the number of points of interest of this Surface.
   *
   * \return The number of points of interest of this Surface.
   */
  size_t getNumPointsOfInterest() const;

  /*!
   * \brief Point of interest value accessor.
   *
   * Get the Point of interest of this Surface at the given index.
   *
   * \param i The index of the point of interest to get.
   * \return The point of interest at the given index.
   * \throws std::out_of_range Thrown if the Point of interest at the given index does not exist.
   */
  const PointOfInterest &getPointOfInterest(const size_t index) const;

  /*!
   * \brief Point of interest adder.
   *
   * Add the Point of interest to this Surface.
   *
   * \param point_of_interest The new point of interest to add.
   */
  void addPointOfInterest(const PointOfInterest &point_of_interest);

  /*!
   * \brief Point of interest remover.
   *
   * Remove the point of interest at the given index. An invalid index results in no effect.
   *
   * \param i The index of the point of interest to remove.
   * \throws std::out_of_range Thrown if the point of interest at the given index does not exist.
   */
  void removePointOfInterest(const size_t index);

private:
  /*! Name of the surface and frame ID. */
  std::string name_, frame_id_;
  /*! Size of the surface. */
  double width_, height_;
  /*! List of name aliases. */
  std::vector<std::string> aliases_;
  /*! List of placement surfaces. */
  std::vector<PlacementSurface> placement_surfaces_;
  /*! List of POIs. */
  std::vector<PointOfInterest> pois_;
};

}
}

#endif

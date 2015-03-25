/*!
 * \file Room.h
 * \brief Room configuration information.
 *
 * A room consists of a series of surfaces.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#ifndef RAIL_INTERACTIVE_WORLD_ROOM_H_
#define RAIL_INTERACTIVE_WORLD_ROOM_H_

#include <interactive_world_tools/Surface.h>
#include <string>

namespace rail
{
namespace interactive_world
{

/*!
 * \class Room
 * \brief Room configuration information.
 *
 * A room consists of a series of surfaces.
 */
class Room
{
public:
  /*!
   * \brief Create a new Room.
   *
   * Create a new Room with the given name and frame ID. The surface list is empty by default.
   *
   * \param name The name of the room (defaults to the empty string).
   * \param frame_id The frame ID of the room (defaults to the empty string).
   */
  Room(const std::string &name = "", const std::string &frame_id = "");

  /*!
   * \brief Name value accessor.
   *
   * Get the name value of this Room.
   *
   * \return The name value.
   */
  const std::string &getName() const;

  /*!
   * \brief Name value mutator.
   *
   * Set the name value of this Room.
   *
   * \param name The new name value.
   */
  void setName(const std::string &name);

  /*!
   * \brief Frame ID value accessor.
   *
   * Get the frame ID value of this Room.
   *
   * \return The frame ID value.
   */
  const std::string &getFrameID() const;

  /*!
   * \brief Frame ID value mutator.
   *
   * Set the frame ID value of this Room.
   *
   * \param frame_id The new frame ID value.
   */
  void setFrameID(const std::string &frame_id);

  /*!
   * \brief Surfaces value accessor.
   *
   * Get the surfaces of this World.
   *
   * \return The surfaces.
   */
  const std::vector<Surface> &getSurfaces() const;

  /*!
   * \brief Surfaces size accessor.
   *
   * Get the number of surfaces of this World.
   *
   * \return The number of surfaces of this World.
   */
  size_t getNumSurfaces() const;

  /*!
   * \brief Surface value accessor.
   *
   * Get the surface of this World at the given index.
   *
   * \param i The index of the Surface to get.
   * \return The surface at the given index.
   * \throws std::out_of_range Thrown if the surface at the given index does not exist.
   */
  const Surface &getSurface(const size_t index) const;

  /*!
   * \brief Surface adder.
   *
   * Add the surface to this World.
   *
   * \param surface The new surface to add.
   */
  void addSurface(const Surface &surface);

  /*!
   * \brief Surface remover.
   *
   * Remove the surface at the given index. An invalid index results in no effect.
   *
   * \param i The index of the surface to remove.
   * \throws std::out_of_range Thrown if the surface at the given index does not exist.
   */
  void removeSurface(const size_t index);

  /*!
   * \brief Surface finder.
   *
   * Find a surface with the given name. This will also check the aliases. Case is not important.
   *
   * \param name The name or alias of the surface to find.
   * \throws std::out_of_range Thrown if no surface with the given name exists.
   */
  const Surface &findSurface(const std::string &name) const;

private:
  /*! Name of the surface and frame ID. */
  std::string name_, frame_id_;
  /*! List of surfaces. */
  std::vector<Surface> surfaces_;
};

}
}

#endif

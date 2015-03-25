/*!
 * \file World.h
 * \brief World configuration information.
 *
 * A world consists of a series of rooms and surfaces. Surfaces can have points of interest as well.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#ifndef RAIL_INTERACTIVE_WORLD_WORLD_H_
#define RAIL_INTERACTIVE_WORLD_WORLD_H_

#include <interactive_world_tools/Room.h>
#include <interactive_world_tools/Surface.h>
#include <vector>

namespace rail
{
namespace interactive_world
{

/*!
 * \class World
 * \brief World configuration information.
 *
 * A world consists of a series of rooms and surfaces. Surfaces can have points of interest as well.
 */
class World
{
public:
  /*!
   * \brief Create a new World.
   *
   * Creates a new empty world.
   */
  World();

  /*!
   * \brief Rooms value accessor.
   *
   * Get the rooms of this World.
   *
   * \return The rooms.
   */
  const std::vector<Room> &getRooms() const;

  /*!
   * \brief Rooms size accessor.
   *
   * Get the number of rooms of this World.
   *
   * \return The number of rooms of this World.
   */
  size_t getNumRooms() const;

  /*!
   * \brief Room pose value accessor.
   *
   * Get the room of this World at the given index.
   *
   * \param i The index of the Room to get.
   * \return The room pose at the given index.
   * \throws std::out_of_range Thrown if the room at the given index does not exist.
   */
  const Room &getRoom(const size_t index) const;

  /*!
   * \brief Room adder.
   *
   * Add the room to this World.
   *
   * \param room The new room to add.
   */
  void addRoom(const Room &room);

  /*!
   * \brief Room remover.
   *
   * Remove the room at the given index. An invalid index results in no effect.
   *
   * \param i The index of the room pose to remove.
   * \throws std::out_of_range Thrown if the room at the given index does not exist.
   */
  void removeRoom(const size_t index);

  /*!
   * \brief Surface finder.
   *
   * Find all surfaces in the world with the given name (or alias).
   *
   * \param name The name (or alias) of the surface to search for.
   * \return A vector of references to the surfaces in the world that were found.
   */
  std::vector<const Surface *> findSurfaces(const std::string &name);

private:
  /*! Room information for the world. */
  std::vector<Room> rooms_;
};

}
}

#endif

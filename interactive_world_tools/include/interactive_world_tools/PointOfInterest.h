/*!
 * \file PointOfInterest.h
 * \brief Point of interest configuration information.
 *
 * A POI contains a name and associated frame.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 18, 2015
 */

#ifndef RAIL_INTERACTIVE_WORLD_POINT_OF_INTEREST_H_
#define RAIL_INTERACTIVE_WORLD_POINT_OF_INTEREST_H_

#include <string>

namespace rail
{
namespace interactive_world
{

/*!
 * \class PointOfInterest
 * \brief Point of interest configuration information.
 *
 * A POI contains a name and associated frame.
 */
class PointOfInterest
{
public:
  /*!
   * \brief Create a new PointOfInterest.
   *
   * Create a new PointOfInterest with the given name and frame ID.
   *
   * \param name The name of the POI (defaults to the empty string).
   * \param frame_id The frame ID of the POI (defaults to the empty string).
   */
  PointOfInterest(const std::string &name = "", const std::string &frame_id = "");

  /*!
   * \brief Name value accessor.
   *
   * Get the name value of this PointOfInterest.
   *
   * \return The name value.
   */
  const std::string &getName() const;

  /*!
   * \brief Name value mutator.
   *
   * Set the  name value of this PointOfInterest.
   *
   * \param name The new name value.
   */
  void setName(const std::string &name);

  /*!
   * \brief Frame ID value accessor.
   *
   * Get the frame ID value of this PointOfInterest.
   *
   * \return The frame ID value.
   */
  const std::string &getFrameID() const;

  /*!
   * \brief Frame ID value mutator.
   *
   * Set the frame ID value of this PointOfInterest.
   *
   * \param frame_id The new frame ID name value.
   */
  void setFrameID(const std::string &frame_id);

private:
  /*! Name of the POI and frame ID. */
  std::string name_, frame_id_;
};

}
}

#endif

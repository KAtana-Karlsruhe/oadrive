#ifndef OADRIVE_TRACKSEGMENT_H_INCLUDED
#define OADRIVE_TRACKSEGMENT_H_INCLUDED

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

#include "Patch.h"
#include "ManagedPose.h"

class TrackSegment
{
public:
  TrackSegment();

  //! \param patch patch from which the information for the TrackSegment will be extracted.
  TrackSegment(Patch::ConstPtr patch, u_int32_t id);

  TrackSegment(Patch::ConstPtr patch, cv::Point startJunction, u_int32_t id);

  //! \param patch patch from which the information for the TrackSegment will be extracted.
  TrackSegment(Patch::ConstPtr patch, u_int32_t id, double matchingValue);

  /*! \param type type of the underlying Patch
   *  \param startJunction the startJunction of the Track in Imagecoordinates
   *  \param endJunction the endJunction of the Track in Imagecoordinates
   *  \param patchLocation the upper left cornder of the Track
   *  \param size height and width of the Track
   */
  TrackSegment(Patch::PatchType type, ManagedPose startJunction, ManagedPose endJunction, ManagedPose patchLocation, cv::Size size, int id);

  //! \return the point where the upper left corner of the track is located
  //cv::Point getPatchLocation() const { return cv::Point(m_patch_location.getPose().getX(), m_patch_location.getPose().getY()); }

  //! \return the pointer where the center of the track is located.
  /*cv::Point getCenter() const
  {
    cv::Point center(getPatchLocation().x + getSize().width/2.0f, getPatchLocation().y + getSize().height/2.0f);
    return center;
  }*/

  //! \return the startJunction of the Track
  ManagedPose getStartJunction() const { return m_start_junction; }

  //! \return the endJunction of the track
  ManagedPose getEndJunction() const { return m_end_junction; }

  //! \return describes whether the the track's matching value is high enough
  bool isValid() const { return m_is_valid; }

  void setValid(bool isValid) { m_is_valid = isValid; }

  //! ID
  u_int32_t getID() const     { return m_id; }
  void setID(u_int32_t id)    { m_id = id; }

  cv::Size getSize() const { return m_size; }

  double getMatchingValue() const   { return m_matching_value; }
  void setMatchingValue(double matchValue) { m_matching_value = matchValue; }

  //! \param delta the x and y translation as a point
  void translate(cv::Point delta);

  /*! \param center the rotation center
   * \param angle the angle in degree
   */
  void rotate(cv::Point center, float angle);

  //! Return type of associated patch
  Patch::PatchType getPatchType() const { return m_patch_type; }
  void setPatchType(Patch::PatchType patch_type) { m_patch_type = patch_type; }

  //! Let start and end junction swap, so that the TrackSegment represents the reverse driving direction
  void flipSegment();

private:

  //! Patch type associtated with this TrackSegment
  Patch::PatchType m_patch_type;

  //! Left upper corner of patch
  //ManagedPose m_patch_location;

  ManagedPose m_start_junction;
  ManagedPose m_end_junction;

  cv::Size m_size;
  bool m_is_valid;
  u_int32_t m_id;
  double m_matching_value;
};

#endif

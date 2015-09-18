#ifndef OADRIVE_VISION_MATCHER_H_INCLUDED
#define OADRIVE_VISION_MATCHER_H_INCLUDED

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

#include <functional>

#include "Patch.h"
#include "TrackSegment.h"

namespace oadrive {
namespace vision  {

//! Enum to specify patches to search for
enum PatchesToLookFor : u_int8_t
{
  DEFAULT  = (u_int8_t)0,
  STRAIGHTS= (u_int8_t)1
};

}
}

class Matcher
{
public:
  //! Patch rotations
  typedef std::vector<Patch::ConstPtr> PatchRotationContainer;
  typedef std::shared_ptr<PatchRotationContainer> PatchRotationContainerPtr;

  //! Search tolerance from last endjunction
  typedef struct { oadrive::core::Pose2d search_pose; double width; double height; } SearchTolerance;

  //! Result from matching
  struct MatchResult
  {
    std::size_t index;
    cv::Point best_loc;
    double best_val;

    bool operator <(const MatchResult& other)
    {
      return best_val < other.best_val;
    }
  };

  //! Convienience pointer
  typedef std::shared_ptr<Matcher> Ptr;

  //! Constructor
  Matcher();

  cv::RotatedRect createSearchspaceForPatch(const SearchTolerance& tolerance, Patch::ConstPtr patch) const;

  /*! \param input the image in which the best patch shall be found.
   *  \param searchspace the possibly rotated area in which the patches are presumed.
   *  \param patches the set of patches which should be possible.
   *  \return The input relative TrackInformation.
   */
  TrackSegment matchPatterns(const cv::Mat& input, const SearchTolerance& tolerance, const std::vector<PatchRotationContainerPtr>& patches);

  /*! \param searchspace
   *  \return returns if given rotated rect fits in bird view image
   *  \param searchspace the given input searchspace.
   *  \param searchspace the searchspace wich should be checked whether it is still in the input image.
   *  \return wether it is on the image or not.
   */
  bool isSearchspaceValid(const cv::RotatedRect& searchspace, const cv::Size& input_picture_size) const;

  bool isSearchspaceInInput(const cv::RotatedRect& searchspace, const cv::Size& input_picture_size) const;

  void clipSearchspace(cv::RotatedRect& searchspace, const cv::Size& input_picture_size) const;

  //! Returns true if point lies within (or on border) of the quadrangle
  static bool isWithinOrOnLineOfQuadrangle(const oadrive::core::Position2d& point, const Quadrangle& q);

  //! Check if the two quadrangles are overlapping
  static bool overlapping(const Quadrangle& q1, const Quadrangle& q2);

  //! Get field of view in pixel coordinates
  const Quadrangle& getFieldOfViewPixel() const   { return m_field_of_vision_pixel; }

  //! Calculate m_field_of_vision_pixel to evalutate searchspaces
  void initializeTrapezoid(const ManagedPose::FieldOfViewPoints& field_of_view_points, float scaling_factor);

  //! Calculate the patch mask
  void createPatchMask(const ManagedPose::FieldOfViewPoints& field_of_view_points);

  //! Get center of field of vision trapezium
  const oadrive::core::Position2d& getFieldOfViewCenter() const   { return m_field_of_view_center; }
private:

  /**
   * @brief matchPatchType
   * @param rotations of patch type with SAME SIZE!
   * @param match_value of winner rotation
   * @return Patch
   */
  Patch::ConstPtr matchPatchType(const cv::Mat& input_image, const PatchRotationContainerPtr& rotations, const SearchTolerance& tolerance, double& match_value, cv::Point& match_location) const;

  /*! \param input the given input image.
   *  \param searchspace the searchspace wich should be checked whether it is still in the input image.
   *  \return wether it is on the image or not.
   */
  bool verifySearchspace(cv::Mat& input, const cv::RotatedRect& searchspace);

  /*! \param input the given input image
   *  \param searchspace the area which is supposed to be cut out.
   * \return the cut out image area.
   */
  cv::Mat cutOut(const cv::Mat& input, const cv::RotatedRect& searchspace) const;

  //! Perform matching on given set of patches
  MatchResult performMatchingOnSubset(const PatchRotationContainerPtr& rotations,
                                      std::size_t from,
                                      std::size_t to,
                                      const cv::Mat& search_image,
                                      cv::Mat& result_mat) const;


  //! Draw Mask on given Mat, representing the actual seen area
  void drawMatchMask(cv::Mat& mask, const cv::RotatedRect &searchspace) const;

  //! Field of vision, pixel coordinates
  Quadrangle m_field_of_vision_pixel;
  //! Center of field of vision
  oadrive::core::Position2d m_field_of_view_center;

  //! Actual field of vision, pixel coordinates
  Quadrangle m_actual_field_of_vision;
};

#endif

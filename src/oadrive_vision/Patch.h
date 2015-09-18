#ifndef OADRIVE_VISION_PATCH_H_INCLUDED
#define OADRIVE_VISION_PATCH_H_INCLUDED

#include <vector>
#include <map>
#include <string>
#include <cmath>
#include<memory>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Searchspace.h"
#include "ManagedPose.h"


/************** LANETRACKER_DEBUG ***************/
//#define KATANA_LT_WRITE_PATCHES
//#define KATANA_LT_WRITE_SEARCHSPACE
//#define KATANA_LT_MATCHER
//#define KATANA_LT_MATCHER_MATCH_VALUES_DEBUG
//#define KATANA_LT_STITCHER
#define KATANA_LT_WRITE_STITCHES
//#define KATANA_LT_WRITE_STITCHES_EVERY_STEP
//#define KATANA_LT_INITIAL_STITCH
//#define KATANA_LT_PATCH_SELECTOR_DEBUG


/*
 * Patches are used to represent segments of a lane in birdview perspective.
 *
 * Every patch has a pattern type and is aware of its rotation around the start
 * junction relative to the base instance of that type. Additionally, every
 * patch has a start and an end junction point, so patches can be 'stitched'
 * together by connecting the start junction of one patch to the end junction of
 * another one.
 */
// enum



class Patch
{
public:
  typedef std::shared_ptr<Patch> Ptr;
  typedef std::shared_ptr<const Patch> ConstPtr;


  //TODO: This should be in common and is. Find a solution and remove this part.
  //! Add type in the patchfactory
  enum PatchType : u_int8_t
  {
    STRAIGHT=0,
    SMALL_L_CURVE=1,
    SMALL_R_CURVE=2,
    JUNCTION=3,
    T_CROSS_LEFT=4,
    T_CROSS_RIGHT=5,
    PATCH_COUNT=6  //< keep this up-to-date!
  };

  // initialised in cpp
  static std::map<PatchType, std::string> PATCH_NAMES;

  // Constructors
  Patch();

  /*! \param pattern
   *  \param type
   *  \param baseStartJunction
   *  \param baseStartJunction
   *  \param baseEndJunction
   *  \param startDirection oriendation at start junction in radian
   *  \param deltaOrientation difference of orienation at end junction in radian
   */
  Patch(const cv::Mat &pattern, PatchType type,
        cv::Point2f baseStartJunction, cv::Point2f baseEndJunction,
        float startDirection = 0.0f, float deltaOrientation = 0.0f);

  Patch(const Patch &patch);

  Patch& operator= (const Patch &patch);

  // Getters
  const cv::Mat& getPattern() const { return m_pattern; }
  const cv::Point2f& getStartJunction() const { return m_start_junction; }
  const cv::Point2f& getEndJunction() const { return m_end_junction; }
  float getStartDirection() const { return m_start_direction; }
  float getEndDirection() const { return m_start_direction + m_delta_orientation; }
  float getDeltaOrientation() const { return m_delta_orientation; }
  cv::Size getSize() const { return cv::Size(m_pattern.cols, m_pattern.rows); }
  PatchType getPatchType() const { return m_type; }

  //! Px/per 0.1mm in patterns on disk
  static const double INITIAL_PICTURE_PIXEL_RATIO;

  //! Return center pose as seen from start junction
  const oadrive::core::Pose2d& getCenterPose() const    { return m_start_to_center; }

private:
  void initializeEndJunction(cv::Point base_end_junction);

  PatchType m_type;
  cv::Point2f m_start_junction;
  float m_start_direction;
  cv::Point2f m_end_junction;
  float m_delta_orientation;
  cv::Mat m_pattern;

  //! Pose of center (direction facing to to top of patch) as seen from start junction coordinate system
  oadrive::core::Pose2d m_start_to_center;
};

#endif

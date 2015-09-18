#ifndef OADRIVE_VISION_STITCHER_H_INCLUDED
#define OADRIVE_VISION_STITCHER_H_INCLUDED

#include <vector>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <tuple>
#include <array>


#include "TrackSegment.h"
#include "Matcher.h"
#include "Patch.h"
#include "PatchFactory.h"
#include "DrawUtil.h"

#include <oadrive_vision/PatchSelector.h>


class Stitcher
{
public:

  //! Constructor
  Stitcher();

  /*! \param input input matrix to find Patches in
   *  \param consistentTrackInformation apriori knowledge about the track in Pixel coordinates
   *  \return a vector of track information according to found patches
   */
  std::vector<TrackSegment> stitch(cv::Mat &input,
                                   const std::vector<TrackSegment>& consistentTrackInformation,
                                   oadrive::vision::PatchSelector::STITCHPROFILE profile,
                                   u_int8_t number_of_stitches = 0,
                                   oadrive::vision::PatchesToLookFor patches_to_look_for = oadrive::vision::PatchesToLookFor::DEFAULT,
                                   double matching_threshold = -1.0
                                   );


  //! initialize patches to search for, returns false on error (patches will be loaded) -- IT IS NECESSARY TO CALL THIS FUNCTION ONCE
  bool initializePatches();

  void updatePose(const oadrive::core::Pose2d& vehicle_pose) {m_vehicle_pose = vehicle_pose; }

  static constexpr double PATCH_MATCHING_THRESHOLD = 0.25;
  static constexpr double CROSS_MATCHING_THRESHOLD = 0.3;

  static const u_int8_t DEFAULT_NUMBER_OF_STITCHES = 2;

private:
  //! PatchSelector: strategy for stitching
  oadrive::vision::PatchSelector m_patch_selector;

  //! matcher used to do template matching
  Matcher::Ptr m_matcher;

  int m_next_unused_id;

  //! Field of vision, car coordiantes
  static const ManagedPose::FieldOfViewPoints FIELD_OF_VISION;
  //! Actaul Field of vision for matcher patch mask, car coordiantes
  static const ManagedPose::FieldOfViewPoints FIELD_OF_VISION_MASK;

  //! Scaling factor for field of view trapezoid
  static constexpr float FIELD_OF_VISION_SCALING = 0.8;

   oadrive::core::Pose2d m_vehicle_pose;
};

#endif

// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-02-24
 *
 */
//----------------------------------------------------------------------

#ifndef _OADRIVE_VISION_PATCHSELECTOR_H
#define _OADRIVE_VISION_PATCHSELECTOR_H


#include <oadrive_vision/Matcher.h>

namespace oadrive {
namespace vision  {


class PatchSelector
{
public:

  //! Enum for patch sets
  enum PatchSet : u_int8_t {
    STANDARD,
    JUNCTIONS,
    STRAIGHTS,
    STRAIGHTS_ROTATED
  };

  //! Enum for searchspace tolerances
  enum STITCHPROFILE : u_int8_t {
    STITCH_NORMAL=0,
    STITCH_INITIAL=1,
    STITCH_EMERGENCY=2,
    STITCH_PROGRESS=3,
    STITCH_JUNCTION=4,
    STITCH_JUNCTION_AGAIN=5,
    STITCH_AFTER_JUNCTION=6,
    STITCH_PARKING_PULL_OUT=7,
    STITCH_MODE_COUNT=8 //< keep this up-to-date
  };

  enum StitchPhase : u_int8_t
  {
    SEARCH_FIRST,
    GO_UP,
    GO_DOWN,
    FINISHED
  };

  //! Constructor
  PatchSelector();

  //! Destructor
  virtual ~PatchSelector();

  //! Set Matcher instance
  void setMatcher(Matcher::Ptr matcher)   { m_matcher = matcher; }

  //! Create patch sets vector, call once
  void createPatchSets();

  //! Set data
  void initializeCurrentFrame(const std::vector<TrackSegment>& consistent_tracks,
                              const cv::Size& input_picture_size,
                              STITCHPROFILE profile,
                              oadrive::vision::PatchesToLookFor patches_to_look_for);

  //! Return everything needed for matcher
  bool getNextSearchData(std::vector<Matcher::PatchRotationContainerPtr>& patch_container, Matcher::SearchTolerance& tolerance);

  //! Advance on step -> determine next search pose, when this is not possible, return false
  bool advanceOneStep();

  //! Add the found match to the current position
  void registerMatch(const TrackSegment& match);

  //! Set PatchSelector for down stitching
  void setDownToStitching()   { m_stitchphase = GO_DOWN; }

  //!
  StitchPhase getStitchPhase() const    { return m_stitchphase; }

  //! return vector with found patches sorted by ID
  std::vector<TrackSegment> getMatches() const;

private:

  //! Create search tolerance to STITCHPROFILE, input_size only needed in emergency case (@todo change this)
  Matcher::SearchTolerance getSearchTolerance(const oadrive::core::Pose2d& target_pose, STITCHPROFILE profile, const cv::Size& input_size);

  //! Get the patches we need to search at @param id in consistent patches vector
  std::vector<Matcher::PatchRotationContainerPtr> getPatchesToLookFor();

  //! search for appropriate pose to start stitching
  bool selectStitcherStartPoseFromKnownPatches(oadrive::core::Pose2d& search_start_pose, size_t& id);

  //! Picture size
  cv::Size m_input_picture_size;

  //! Consistend tracks obtained from mission control
  std::vector<TrackSegment> m_initial_guess;

  //! Found matches
  std::map<size_t, TrackSegment> m_found_matches;

  //! Mission control start stitching profile
  STITCHPROFILE m_start_profile;

  //! Patch sets
  std::map<u_int8_t, Matcher::PatchRotationContainerPtr> m_patch_sets;

  //! Matcher
  Matcher::Ptr m_matcher;

  //! Mode of patch selector -> search for initial start pose, up-stitching, down-stitching
  StitchPhase m_stitchphase;

  //! The current pose where to search
  oadrive::core::Pose2d m_current_search_pose;

  //! The start id from initial guess in this frame
  size_t m_start_id;
  //! The id of the initial guess we're currently looking for
  size_t m_id_of_guess;

  //! The (mission control) id of the current searched patch
  size_t m_current_patch_id;
  
  //! Allowed Patch types
  oadrive::vision::PatchesToLookFor m_patches_to_look_for;
};


} // vision
} // oadrive

#endif //_OADRIVE_VISION_PATCHSELECTOR_H

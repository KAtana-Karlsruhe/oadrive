#include "Stitcher.h"
#include <sstream>

using namespace oadrive::core;

// the clockwise order of the corner points is important for evaluating overlapping,
// transfrom to Pixel Coordinates will swap x/y, so that the resulting pixel coordinate
// order is counter clockwise (math. pos.)
const ManagedPose::FieldOfViewPoints Stitcher::FIELD_OF_VISION = std::array<ManagedPose, 4>
{
  ManagedPose(25000, 13500, 0, ManagedPose::Context::IN_CAR),   // up left
  ManagedPose(25000, -13500, 0, ManagedPose::Context::IN_CAR),  // up right
  ManagedPose(10000, -2600, 0, ManagedPose::Context::IN_CAR),    // down right
  ManagedPose(10000, 2600, 0, ManagedPose::Context::IN_CAR)      // down left
};

const ManagedPose::FieldOfViewPoints Stitcher::FIELD_OF_VISION_MASK = std::array<ManagedPose, 4>
{
  ManagedPose(32000, 14800, 0, ManagedPose::Context::IN_CAR),   // up left
  ManagedPose(32000, -14800, 0, ManagedPose::Context::IN_CAR),  // up right
  ManagedPose(7400, -2600, 0, ManagedPose::Context::IN_CAR),    // down right
  ManagedPose(7400, 2600, 0, ManagedPose::Context::IN_CAR)      // down left
};

Stitcher::Stitcher()
{
  m_matcher.reset(new Matcher());
  m_patch_selector.setMatcher(m_matcher);
}

bool Stitcher::initializePatches()
{
  // try to initialze patchfactory by loading patches from disk
  if (!PatchFactory::getFactory()->initialize())
    return false;

  // calculate field of view trapezoid in pixel coordinates, use hard-coded values (FIELD_OF_VISION and FIELD_OF_VISION_SCALING)
  m_matcher->initializeTrapezoid(FIELD_OF_VISION, FIELD_OF_VISION_SCALING);

  // create patch mask for matcher
  m_matcher->createPatchMask(FIELD_OF_VISION_MASK);

  // initialize patch sets
  m_patch_selector.createPatchSets();

  return true;
}

std::vector<TrackSegment> Stitcher::stitch(cv::Mat& input,
                                           const std::vector<TrackSegment>& consistentTrackInformation,
                                           oadrive::vision::PatchSelector::STITCHPROFILE profile,
                                           u_int8_t number_of_stitches,
                                           oadrive::vision::PatchesToLookFor patches_to_look_for,
                                           double matching_threshold)
{
  assert(input.cols > 0 && input.rows > 0 && "input in stitcher null");

  // initialize matching_threshold depending on profile and matching threshold set from outside
  if (matching_threshold < 0)
  {
    if (profile == oadrive::vision::PatchSelector::STITCHPROFILE::STITCH_JUNCTION
     || profile == oadrive::vision::PatchSelector::STITCHPROFILE::STITCH_JUNCTION_AGAIN)
    {
      matching_threshold = CROSS_MATCHING_THRESHOLD;
    }
    else
    {
      matching_threshold = PATCH_MATCHING_THRESHOLD;
    }
  }

  if (number_of_stitches == 0)
    number_of_stitches = DEFAULT_NUMBER_OF_STITCHES;

  //Generate debug Image
#ifdef KATANA_LT_WRITE_STITCHES
  cv::Mat output;
  cv::Mat output2;
  input.copyTo(output2);
  cv::cvtColor(output2, output, CV_GRAY2BGR);
#endif

  // init Patch selector for current frame
  m_patch_selector.initializeCurrentFrame(consistentTrackInformation, input.size(), profile, patches_to_look_for);

  for (u_int8_t i = 0; i < number_of_stitches; i++)
  {
    // info what and where to search
    Matcher::SearchTolerance tolerance;
    std::vector<Matcher::PatchRotationContainerPtr> search_patches;

    if (!m_patch_selector.getNextSearchData(search_patches, tolerance))
    {
      break;
    }


#ifdef KATANA_LT_WRITE_STITCHES
    DrawUtil::drawSearchTolerance(output, tolerance);
    Patch::ConstPtr debugPatch = PatchFactory::getFactory()->getStraightBasePatch();
    if (profile == oadrive::vision::PatchSelector::STITCHPROFILE::STITCH_JUNCTION
    || profile == oadrive::vision::PatchSelector::STITCHPROFILE::STITCH_JUNCTION_AGAIN)
      debugPatch = PatchFactory::getFactory()->getPatchByType(Patch::PatchType::JUNCTION, 0);
    DrawUtil::drawSearchspace(output, m_matcher->createSearchspaceForPatch(tolerance, debugPatch));
#endif
#ifdef KATANA_LT_WRITE_STITCHES_EVERY_STEP
    DrawUtil::writeImageToFile(output, "_stitched");
#endif

    // matching
    TrackSegment match = m_matcher->matchPatterns(input, tolerance, search_patches);

    // check for threshold, stop if bad
    if (match.getMatchingValue() < matching_threshold)
    {
      #ifdef KATANA_LT_WRITE_STITCHES
        // failed match -> draw searchspace in magenta
        DrawUtil::drawSearchspace(output, m_matcher->createSearchspaceForPatch(tolerance, debugPatch), 3);
        if (match.isValid() && match.getPatchType() < Patch::PATCH_COUNT)
          DrawUtil::drawTrack(output, match, match.getID());
      #endif
      break;
    }

    // tell PatchSelector what match to add
    m_patch_selector.registerMatch(match);

    // tell Patch selector to step forward, if this is not possible (e.g. newly discovered junction, stop stitching
    if (!m_patch_selector.advanceOneStep())
    {
      break;
    }

  }

  std::vector<TrackSegment> matches = m_patch_selector.getMatches();
#ifdef KATANA_LT_WRITE_STITCHES
  for (std::vector<TrackSegment>::const_iterator it = matches.begin(); it != matches.end(); it++)
  {
    DrawUtil::drawTrack(output, *it, it->getID());
  }

  // draw IMU
  ManagedPose imu(0,0,0, ManagedPose::Context::IN_CAR);
  ManagedPose imuPixel = ManagedPose::transformCarToPixel(imu);

  DrawUtil::drawManagedPose(output, imuPixel);
  DrawUtil::putTextAt(output, cv::Point(imuPixel.getX() + 5, imuPixel.getY()), "IMU", 3);

  // print odometry
  std::stringstream ss;
  ss << m_vehicle_pose;
  DrawUtil::putTextAt(output, cv::Point(20, 20), ss.str(), 3);


  DrawUtil::writeImageToFile(output, "_stitched");
#endif
  for (std::vector<TrackSegment>::iterator it = matches.begin(); it != matches.end(); it++)
  {
    // TODO: add assert in Lanetracker.cpp in AADC repo!
    // only send out junctions of type JUNCTION!
    if ((*it).getPatchType() == Patch::PatchType::T_CROSS_LEFT || (*it).getPatchType() == Patch::PatchType::T_CROSS_RIGHT)
    {
      (*it).setPatchType(Patch::PatchType::JUNCTION);
    }
  }

  return matches;
}

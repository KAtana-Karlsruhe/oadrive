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


#include <oadrive_vision/PatchSelector.h>

#include <oadrive_vision/PatchFactory.h>

using namespace oadrive::core;

namespace oadrive {
namespace vision  {

PatchSelector::PatchSelector()
  : m_stitchphase(SEARCH_FIRST)
{

}
PatchSelector::~PatchSelector()
{

}

void PatchSelector::createPatchSets()
{
  // rotations in degree...
  static const double c = M_PI/180.0f;


  std::vector<double> rotations;
  rotations.push_back(-8*c);
  rotations.push_back(-6*c);
  rotations.push_back(-4*c);
  rotations.push_back(-2*c);
  rotations.push_back(0*c);
  rotations.push_back(2*c);
  rotations.push_back(4*c);
  rotations.push_back(6*c);
  rotations.push_back(8*c);

  // Standard road types
  std::vector<Patch::PatchType> types;

  types.push_back(Patch::STRAIGHT);
  types.push_back(Patch::SMALL_L_CURVE);
  types.push_back(Patch::SMALL_R_CURVE);


  // Initialize standard rotated patches
  m_patch_sets[STANDARD].reset(new Matcher::PatchRotationContainer);

  for (std::vector<Patch::PatchType>::const_iterator type_it = types.begin(); type_it != types.end(); ++type_it)
  {
    for (std::vector<double>::const_iterator rotation_it = rotations.begin(); rotation_it!=rotations.end(); ++rotation_it)
    {
      m_patch_sets[STANDARD]->push_back(PatchFactory::getFactory()->getPatchByType(*type_it, *rotation_it));
    }
  }

  // Initialize junction patches

  std::vector<Patch::PatchType> junction_types;

  junction_types.push_back(Patch::JUNCTION);
  junction_types.push_back(Patch::T_CROSS_LEFT);
  junction_types.push_back(Patch::T_CROSS_RIGHT);
  m_patch_sets[JUNCTIONS].reset(new Matcher::PatchRotationContainer);

  std::vector<double> junction_rotations;

  for(int i = -30; i <= 30; ++i)
  {
    junction_rotations.push_back(i*c);
  }

  junction_rotations.push_back(-45*c);
  junction_rotations.push_back(-40*c);
  junction_rotations.push_back(-35*c);
  junction_rotations.push_back(35*c);
  junction_rotations.push_back(40*c);
  junction_rotations.push_back(45*c);

  for (std::vector<Patch::PatchType>::const_iterator type_it = junction_types.begin(); type_it != junction_types.end(); ++type_it)
  {
    for (std::vector<double>::const_iterator rotation_it = junction_rotations.begin(); rotation_it != junction_rotations.end(); ++rotation_it)
    {
      m_patch_sets[JUNCTIONS]->push_back(PatchFactory::getFactory()->getPatchByType(*type_it, *rotation_it));
    }
  }

  // Initialize straight patches

  m_patch_sets[STRAIGHTS].reset(new Matcher::PatchRotationContainer);

  std::vector<double> straight_rotations;

  for(int8_t i = -8; i <= 8; ++i)
  {
    straight_rotations.push_back(i*c);
  }

  for (std::vector<double>::const_iterator rotation_it = straight_rotations.begin(); rotation_it != straight_rotations.end(); ++rotation_it)
  {
    m_patch_sets[STRAIGHTS]->push_back(PatchFactory::getFactory()->getPatchByType(Patch::STRAIGHT, *rotation_it));
  }

  // Initialize straight patches with a lot of rotations

  m_patch_sets[STRAIGHTS_ROTATED].reset(new Matcher::PatchRotationContainer);

  straight_rotations.clear();

  for(int8_t i = -30; i <= 30; ++i)
  {
    straight_rotations.push_back(i*c);
  }

  for (std::vector<double>::const_iterator rotation_it = straight_rotations.begin(); rotation_it != straight_rotations.end(); ++rotation_it)
  {
    m_patch_sets[STRAIGHTS_ROTATED]->push_back(PatchFactory::getFactory()->getPatchByType(Patch::STRAIGHT, *rotation_it));
  }

}

void PatchSelector::initializeCurrentFrame(const std::vector<TrackSegment>& consistent_tracks,
                                           const cv::Size& input_picture_size,
                                           STITCHPROFILE profile,
                                           PatchesToLookFor patches_to_look_for)
{
  // save data
  m_input_picture_size = input_picture_size;

  m_initial_guess = consistent_tracks;
  m_found_matches.clear();

  m_start_profile = profile;

  // set flag
  m_stitchphase = SEARCH_FIRST;
  
  //
  m_patches_to_look_for = patches_to_look_for;
}

bool PatchSelector::selectStitcherStartPoseFromKnownPatches(oadrive::core::Pose2d& search_start_pose, size_t& id)
{
  assert(!m_initial_guess.empty() && "!consistentTrackInformation.empty()");

  // save visible start poses
  std::vector<std::pair<size_t, oadrive::core::Pose2d> > visible_elements;


  size_t vector_id_counter = 0;
  for (std::vector<TrackSegment>::const_iterator t_it = m_initial_guess.begin(); t_it!=m_initial_guess.end(); ++t_it)
  {
    // assure we only get JUNCTIONS from mission control (no t_crosses!)
    if(m_start_profile == STITCH_JUNCTION || m_start_profile == STITCH_JUNCTION_AGAIN)
    {
      assert(t_it->getPatchType() == Patch::PatchType::JUNCTION);
    }

    // only allow junction tracks if we are in junction mode!
    if(t_it->getPatchType() == Patch::PatchType::JUNCTION)
    {
       assert((m_start_profile == STITCH_JUNCTION
            || m_start_profile == STITCH_JUNCTION_AGAIN)
            && "PatchType is JUNCTION but profile is not STITCH_JUNCTION");
    }

    Matcher::SearchTolerance tolerance = getSearchTolerance(t_it->getStartJunction().getPose().getPose(), m_start_profile, m_input_picture_size);

    // create actual search space with last patch type to check if visible
    cv::RotatedRect searchspace = m_matcher->createSearchspaceForPatch(tolerance, PatchFactory::getFactory()->getPatchByType(t_it->getPatchType(), 0.0));

    if (m_matcher->isSearchspaceValid(searchspace, m_input_picture_size))
    {
      #ifdef KATANA_LT_STITCHER
        std::cout << "Found consistent searchspace!" << std::endl;
      #endif
      visible_elements.push_back(std::pair<size_t, oadrive::core::Pose2d>(vector_id_counter, t_it->getStartJunction().getPose().getPose()));
    }
    else if (!visible_elements.empty())  //< assume that poses further away won't be seen
      break;

    ++vector_id_counter;
  }

  if (vector_id_counter == m_initial_guess.size())  // search was completed until last patch, there is a change that the last endpose is feasible
  {
    // never use end junction of any junction patch
    if(m_initial_guess.back().getPatchType() != Patch::PatchType::JUNCTION
    && m_initial_guess.back().getPatchType() != Patch::PatchType::T_CROSS_LEFT
    && m_initial_guess.back().getPatchType() != Patch::PatchType::T_CROSS_RIGHT)
    {
      // finally check end-junction of last consistent patch
      Matcher::SearchTolerance tolerance = getSearchTolerance(m_initial_guess.back().getEndJunction().getPose().getPose(), m_start_profile, m_input_picture_size);

      // create actual search space with standard patch type to check if visible
      cv::RotatedRect searchspace = m_matcher->createSearchspaceForPatch(tolerance, PatchFactory::getFactory()->getStraightBasePatch());

      if (m_matcher->isSearchspaceValid(searchspace, m_input_picture_size))
      {
        #ifdef KATANA_LT_STITCHER
          std::cout << "Found consistent searchspace!" << std::endl;
        #endif
        // we add a vector id counter not existing in consistend patches vector
        visible_elements.push_back(std::pair<size_t, oadrive::core::Pose2d>(vector_id_counter, m_initial_guess.back().getEndJunction().getPose().getPose()));
      }
    }
  }

  // nothing found
  if (visible_elements.empty())
    return false;

  // look for start pose nearest to field of view center
  double best_distance = std::numeric_limits<double>::infinity();

  const Position2d target_search_pose = m_matcher->getFieldOfViewCenter() + Position2d(0, 90*100 * ManagedPose::getPixelRatio());

  for (size_t i = 0; i < visible_elements.size(); i++)
  {
    double dist = (visible_elements[i].second.translation() - target_search_pose).norm();
    if (dist < best_distance)
    {
      best_distance = dist;
      search_start_pose = visible_elements[i].second;
      id = visible_elements[i].first;
    }
  }

  return true;
}

bool PatchSelector::advanceOneStep()
{
  assert(m_stitchphase != SEARCH_FIRST && "m_stitchphase != SEARCH_FIRST");
  assert(!m_initial_guess.empty() && "!m_consistent_tracks.empty()");
  assert(!m_found_matches.empty() && "!m_found_matches.empty()");

  if (m_stitchphase == FINISHED)
    return false;

  // don't stitch if we're trying to localize a junction
  if (m_start_profile == STITCH_JUNCTION || m_start_profile == STITCH_JUNCTION_AGAIN)
  {
    return false;
  }

  // try one search down stitch..
  if (m_stitchphase == GO_DOWN)
  {
    if (m_start_id == 0)    //< cannot stitch to a more negative id
      return false;

    Pose2d turn_around;
    PoseTraits<Pose2d>::fromPositionAndOrientationRPY(turn_around, 0, 0, M_PI);

    m_current_search_pose = m_found_matches[m_start_id].getStartJunction().getPose().getPose() * turn_around;
    m_id_of_guess = m_start_id - 1;

    return true;
  }

  // set search pose for next stitch to end pose of current match
  m_current_search_pose = m_found_matches[m_id_of_guess].getEndJunction().getPose().getPose();

  // advance in vector
  m_id_of_guess++;

  return true;
}

void PatchSelector::registerMatch(const TrackSegment& match)
{
  assert(m_stitchphase != SEARCH_FIRST && "m_stitchphase != SEARCH_FIRST");

  TrackSegment add_new;

  add_new = match;
  add_new.setID(m_current_patch_id);

  if (m_stitchphase == GO_DOWN)
  {
    add_new.flipSegment();
  }

  // add the found match to our found match vector or overwrite this position
  m_found_matches[m_id_of_guess] = add_new;

}

bool PatchSelector::getNextSearchData(std::vector<Matcher::PatchRotationContainerPtr>& patch_container, Matcher::SearchTolerance& tolerance)
{
  // PatchSelector has to search for appropriate pose, when this is the first call in a new frame
  if (m_stitchphase == SEARCH_FIRST)
  {
    if (!selectStitcherStartPoseFromKnownPatches(m_current_search_pose, m_start_id))
      return false;

    // success, start pose and id are set
    m_stitchphase = GO_UP;
    m_id_of_guess = m_start_id;
    // set to id of last patch from mission control as default
    m_current_patch_id = m_initial_guess.back().getID();
  }
  else  // last search pose is already set in advanceOneMatch()
  {
    // stitch in progress mode
    m_start_profile = STITCH_PROGRESS;
  }

  // calculate tolerance
  tolerance = getSearchTolerance(m_current_search_pose, m_start_profile, m_input_picture_size);

  // obtain patches to look for at the current position in the consistent patches
  patch_container = getPatchesToLookFor();

  if (patch_container.empty())
    return false;

  // set next_id to PATCH(!) id for next patch (== id from current track segment)
  if (m_id_of_guess < m_initial_guess.size())  //< track segment from mission control -> use this ID
  {
    m_current_patch_id = m_initial_guess.at(m_id_of_guess).getID();
  }
  else  //< increase ID of last track segment by one
  {
    ++m_current_patch_id;
  }

  if (m_stitchphase == GO_DOWN)
    m_stitchphase = FINISHED;

  return true;

}

std::vector<Matcher::PatchRotationContainerPtr> PatchSelector::getPatchesToLookFor()
{
  std::vector<Matcher::PatchRotationContainerPtr> patches;

  assert(!(m_start_profile == STITCH_JUNCTION || m_start_profile == STITCH_JUNCTION_AGAIN) || m_patches_to_look_for != oadrive::vision::STRAIGHTS);
  
  // if mission control wants only straight patches, search only for straight patches
  if (m_patches_to_look_for == oadrive::vision::STRAIGHTS)
  {
    patches.push_back(m_patch_sets[STRAIGHTS]);
    return patches;
  }
  
  // only search for junctions if we have to localize junctions
  if (m_start_profile == STITCH_JUNCTION || m_start_profile == STITCH_JUNCTION_AGAIN)
  {
    patches.push_back(m_patch_sets[JUNCTIONS]);
    return patches;
  }

  if (m_start_profile == STITCH_PARKING_PULL_OUT)
  {
    patches.push_back(m_patch_sets[STRAIGHTS_ROTATED]);
    return patches;
  }

  // search for standard set else
  patches.push_back(m_patch_sets[STANDARD]);
  return patches;
}

Matcher::SearchTolerance PatchSelector::getSearchTolerance(const oadrive::core::Pose2d& target_pose, STITCHPROFILE profile, const cv::Size& input_size)
{
  assert(profile < STITCH_MODE_COUNT);

  Matcher::SearchTolerance tolerance;

  if (profile == STITCHPROFILE::STITCH_EMERGENCY)
  {
    tolerance.search_pose.translation() = m_matcher->getFieldOfViewCenter();
    oadrive::core::PoseTraits<oadrive::core::Pose2d>::fromOrientationRPY(tolerance.search_pose, 0.0);
  }
  else
  {
    tolerance.search_pose = target_pose;
  }

  switch(profile)
  {
  case STITCHPROFILE::STITCH_NORMAL:
      tolerance.width = 20*ManagedPose::getPixelRatio()*100.0f;
      tolerance.height = 3*ManagedPose::getPixelRatio()*100.0f;
      break;
  case STITCHPROFILE::STITCH_INITIAL:
      tolerance.width = 35*ManagedPose::getPixelRatio()*100.0f;
      tolerance.height = 60*ManagedPose::getPixelRatio()*100.0f;
      break;
  case STITCHPROFILE::STITCH_EMERGENCY:
    //! @todo TODO TODO scale properly
      tolerance.width = input_size.width*0.5;
      tolerance.height = input_size.height*0.5;
      break;
  case STITCHPROFILE::STITCH_PROGRESS:
      tolerance.width = 12*ManagedPose::getPixelRatio()*100.0f;
      tolerance.height = 12*ManagedPose::getPixelRatio()*100.0f;
      break;
  case STITCHPROFILE::STITCH_JUNCTION:
      tolerance.width = 70*ManagedPose::getPixelRatio()*100.0f;
      tolerance.height = 70*ManagedPose::getPixelRatio()*100.0f;
      break;
  case STITCHPROFILE::STITCH_JUNCTION_AGAIN:
      tolerance.width = 70*ManagedPose::getPixelRatio()*100.0f;
      tolerance.height = 110*ManagedPose::getPixelRatio()*100.0f;
      break;
  case STITCHPROFILE::STITCH_AFTER_JUNCTION:
      tolerance.width = 50*ManagedPose::getPixelRatio()*100.0f;
      tolerance.height = 50*ManagedPose::getPixelRatio()*100.0f;
      break;
  case STITCHPROFILE::STITCH_PARKING_PULL_OUT:
      tolerance.width = 90*ManagedPose::getPixelRatio()*100.0f;
      tolerance.height = 30*ManagedPose::getPixelRatio()*100.0f;
      break;
  default:
    assert(false && "default switch(profile)");
  }

  return tolerance;
}

std::vector<TrackSegment> PatchSelector::getMatches() const
{
  std::vector<TrackSegment> ret;

  for( std::map<size_t, TrackSegment>::const_iterator it = m_found_matches.begin(); it != m_found_matches.end(); ++it )
  {
    ret.push_back( it->second );
  }

  return ret;
}

} // vision
} // oadrive

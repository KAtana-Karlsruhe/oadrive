#include "TrackSegment.h"

using namespace oadrive::core;

TrackSegment::TrackSegment()
{
  m_matching_value = 0.0;
  m_id = 0;
}

TrackSegment::TrackSegment(Patch::PatchType type, ManagedPose startJunction, ManagedPose endJunction, ManagedPose patchLocation, cv::Size size, int id)
{
  m_matching_value = 0.0;
  m_patch_type = type;
  m_start_junction = startJunction;
  m_end_junction = endJunction;
  //m_patch_location = patchLocation;
  m_size = size;
  m_is_valid = true;
  m_id = id;
}


TrackSegment::TrackSegment(Patch::ConstPtr patch, u_int32_t id)
{
  // TODO: rename Junction to pose.
  m_matching_value = 0.0;
  m_patch_type = patch->getPatchType();
  m_start_junction = ManagedPose(patch->getStartJunction().x, patch->getStartJunction().y, patch->getStartDirection(), ManagedPose::IN_PIXEL);
  m_end_junction = ManagedPose(patch->getEndJunction().x, patch->getEndJunction().y, patch->getEndDirection(), ManagedPose::IN_PIXEL);
  //m_patch_location = ManagedPose(0, 0, 0, ManagedPose::IN_PIXEL);
  m_size = patch->getSize();
  m_is_valid = true;
  m_id = id;
}

TrackSegment::TrackSegment(Patch::ConstPtr patch, u_int32_t id, double matchingValue)
{
  // TODO: rename Junction to pose.
  m_matching_value = matchingValue;
  m_patch_type = patch->getPatchType();
  m_start_junction = ManagedPose(patch->getStartJunction().x, patch->getStartJunction().y, patch->getStartDirection(), ManagedPose::IN_PIXEL);
  m_end_junction = ManagedPose(patch->getEndJunction().x, patch->getEndJunction().y, patch->getEndDirection(), ManagedPose::IN_PIXEL);
  //m_patch_location = ManagedPose(0, 0, 0, ManagedPose::IN_PIXEL);
  m_size = patch->getSize();
  m_is_valid = true;
  m_id = id;
}

TrackSegment::TrackSegment(Patch::ConstPtr patch, cv::Point targetStartJunction, u_int32_t id)
{
  m_matching_value = 0.0;
  m_patch_type = patch->getPatchType();
  m_start_junction = ManagedPose(patch->getStartJunction().x, patch->getStartJunction().y, patch->getStartDirection(), ManagedPose::IN_PIXEL);
  m_end_junction = ManagedPose(patch->getEndJunction().x, patch->getEndJunction().y, patch->getEndDirection(), ManagedPose::IN_PIXEL);
  //m_patch_location = ManagedPose(0, 0, 0, ManagedPose::IN_PIXEL);
  m_size = patch->getSize();
  m_is_valid = true;
  m_id = id;
  // put our start junction to target start junction
  cv::Point delta = targetStartJunction - cv::Point(m_start_junction.getX(), m_start_junction.getY());
  this->translate(delta);
}

void TrackSegment::translate(cv::Point delta)
{
  ManagedPose pose(delta.x, delta.y, 0, ManagedPose::IN_PIXEL);
  //m_patch_location += pose;
  m_start_junction += pose;
  m_end_junction += pose;
}

void TrackSegment::rotate(cv::Point center, float angle)
{
  float rotationInRad = M_PI*angle/180.0f;
  ManagedPose pose(center.x, center.y, rotationInRad, ManagedPose::IN_PIXEL);

  //m_patch_location = m_patch_location.rotate(pose);
  m_start_junction = m_start_junction.rotate(pose);
  m_end_junction = m_end_junction.rotate(pose);
}

void TrackSegment::flipSegment()
{
  Pose2d rotate;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(rotate, 0, 0, M_PI);

  Pose2d tmp;
  tmp = m_start_junction.getPose().getPose() * rotate;

  m_start_junction.getPose().getPose() = m_end_junction.getPose().getPose() * rotate;
  m_end_junction.getPose().getPose() = tmp;

  // patch type has to get switched
  if (m_patch_type == Patch::SMALL_L_CURVE)
    m_patch_type = Patch::SMALL_R_CURVE;
  else if (m_patch_type == Patch::SMALL_R_CURVE)
    m_patch_type = Patch::SMALL_L_CURVE;
}

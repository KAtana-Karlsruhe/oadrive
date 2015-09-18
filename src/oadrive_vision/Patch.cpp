#include "Patch.h"
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;
using namespace oadrive::core;

const double Patch::INITIAL_PICTURE_PIXEL_RATIO = 0.03;
map<Patch::PatchType, string> createPatchNameMap()
{
  map<Patch::PatchType, string> m;
  m[Patch::STRAIGHT] = "straight";
  m[Patch::SMALL_L_CURVE]= "small_l_curve";
  m[Patch::SMALL_R_CURVE]= "small_r_curve";
  m[Patch::JUNCTION]= "cross";
  m[Patch::T_CROSS_LEFT]= "t_cross_left";
  m[Patch::T_CROSS_RIGHT]= "t_cross_right";
  return m;
}

// initialisation of static member PATCH_NAMES
map<Patch::PatchType, string> Patch::PATCH_NAMES = createPatchNameMap();

// Public
Patch::Patch() {
}

Patch::Patch(const Patch &patch)
{
  // deep copy of matrices!
  patch.getPattern().copyTo(m_pattern);

  m_type = patch.m_type;
  m_start_junction = patch.m_start_junction;
  m_end_junction = patch.m_end_junction;
  m_start_direction = patch.m_start_direction;
  m_delta_orientation = patch.m_delta_orientation;
}


Patch::Patch(const Mat &pattern, Patch::PatchType type,
             Point2f baseStartJunction, Point2f baseEndJunction,
             float startDirection, float deltaOrientation)
{
  pattern.copyTo(m_pattern);
  if(ManagedPose::getPixelRatio()!= INITIAL_PICTURE_PIXEL_RATIO)
  {
    cv::resize(m_pattern, m_pattern, Size(),ManagedPose::getPixelRatio()/INITIAL_PICTURE_PIXEL_RATIO, ManagedPose::getPixelRatio()/INITIAL_PICTURE_PIXEL_RATIO, INTER_AREA);
  }
  m_type = type;
  m_start_junction = baseStartJunction;
  m_start_direction = startDirection;
  m_delta_orientation = deltaOrientation;
  initializeEndJunction(baseEndJunction);

  // calculate center pose as seen from start junction (to calculate searchspace)
  Pose2d start_j;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(start_j, m_start_junction.x, m_start_junction.y, 0.0);

  Pose2d center;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(center, getSize().width*0.5, getSize().height*0.5, 0.0);

  m_start_to_center = start_j.inverse() * center;
}


Patch& Patch::operator= (const Patch &patch)
{
  // deep copy of matrices!
  patch.getPattern().copyTo(m_pattern);

  m_type = patch.m_type;
  m_start_junction = patch.m_start_junction;
  m_end_junction = patch.m_end_junction;
  m_start_direction = patch.m_start_direction;
  m_delta_orientation = patch.m_delta_orientation;

  return *this;
}

void Patch::initializeEndJunction(Point base_end_junction)
{
  // rotate baseEndJunction around m_startJunction
  Point center = m_start_junction;
  float rotation = m_start_direction;

  // compute in Point2f to reduce computation errors
  Point2f rotated(base_end_junction.x, base_end_junction.y);

  // rotate point by rotationInRad around center
  if (m_start_direction != 0) {
    // first, translate the point
    Point2f translated = Point2f(base_end_junction.x, base_end_junction.y) - Point2f(center.x, center.y);

    // rotate it as usual
    rotated.x = translated.x * std::cos(rotation) - translated.y * std::sin(rotation);
    rotated.y = translated.x * std::sin(rotation) + translated.y * std::cos(rotation);

    // translate back
    rotated += Point2f(center.x, center.y);
  }
  m_end_junction = rotated;
}

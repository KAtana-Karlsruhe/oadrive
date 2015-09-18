#include "Matcher.h"
#include "PatchFactory.h"
#include "DrawUtil.h"
#include <oadrive_core/Utility.h>

#include <future>

#ifdef KATANA_LT_MATCHER
static int z = 0;
#endif

using namespace oadrive::core;

Matcher::Matcher()
{
}

Patch::ConstPtr Matcher::matchPatchType(const cv::Mat& input_image, const PatchRotationContainerPtr& rotations,
                                        const SearchTolerance& tolerance, double& match_value, cv::Point& match_location) const
{
  assert(!rotations->empty() && "!rotations.empty()");

  int result_cols = -1;
  int result_rows = -1;

  // cut out from actual image
  cv::Mat search_image;
  cv::RotatedRect searchspace;

  // create searchspace for current patch
  searchspace = createSearchspaceForPatch(tolerance, rotations->front());

  // check if searchspace is valid (any overlapping with actual view)
  if (!isSearchspaceValid(searchspace, input_image.size()))
    return Patch::ConstPtr();     // we cannot see this type of patch in any rotation

  //
  if (!isSearchspaceInInput(searchspace, input_image.size()))
  {
    std::cout << "[Matcher] clipping searchspace because border is too small!" << std::endl;
    clipSearchspace(searchspace, input_image.size());
  }

  // actual cutout image for searching
  search_image = cutOut(input_image, searchspace);

#ifdef KATANA_LT_MATCHER
  cv::Mat cut;
  cv::Mat cut2;
  search_image.copyTo(cut2);
  cvtColor(cut2, cut, CV_GRAY2BGR);
  DrawUtil::writeImageToFile(cut, "_" + oadrive::core::to_string(z) + "_cut");
#endif

  // it has to be guaranteed that all patches in rotations have the same size
  cv::Mat result_mat;
  // result matrix is missing one column and one row
  result_cols = search_image.cols - rotations->front()->getPattern().cols + 1;
  result_rows = search_image.rows - rotations->front()->getPattern().rows + 1;
  result_mat.create(result_cols, result_rows, CV_32FC1);

  if (rotations->size() < 15)   //< do single thread matching
  {
    MatchResult match_result = performMatchingOnSubset(rotations, 0, rotations->size(), search_image, result_mat);

    // winner patch
    match_location = match_result.best_loc;
    match_value = match_result.best_val;
    return (*rotations).at(match_result.index);
  }
  else if (rotations->size() < 30)
  {

    // split on two threads
    std::size_t split = rotations->size() / 2;

    //second result mat
    cv::Mat result_mat_second;
    result_mat_second.create(result_cols, result_rows, CV_32FC1);

    // match async
    std::future<MatchResult> future_match = std::async(std::launch::async,
                                                       &Matcher::performMatchingOnSubset,
                                                       this,
                                                       std::cref(rotations),
                                                       0,
                                                       split,
                                                       std::cref(search_image),
                                                       std::ref(result_mat));

    // do second part of matching here
    MatchResult result_second = performMatchingOnSubset(rotations, split, rotations->size(), search_image, result_mat_second);

    // read async results
    const MatchResult result = future_match.get();

    // compare
    if (result.best_val > result_second.best_val)
    {
      match_location = result.best_loc;
      match_value = result.best_val;
      return (*rotations).at(result.index);
    }

    match_location = result_second.best_loc;
    match_value = result_second.best_val;
    return (*rotations).at(result_second.index);
  }

  // MATCH JUNCTIONS ETC, 3 threads

  // split on two threads
  std::size_t split_one = rotations->size() / 3;
  std::size_t split_two = 2*split_one;

  //second result mat
  cv::Mat result_mat_second;
  cv::Mat result_mat_third;
  result_mat_second.create(result_cols, result_rows, CV_32FC1);
  result_mat_third.create(result_cols, result_rows, CV_32FC1);

  // match async
  std::future<MatchResult> future_match_second = std::async(std::launch::async,
                                                     &Matcher::performMatchingOnSubset,
                                                     this,
                                                     std::cref(rotations),
                                                     0,
                                                     split_one,
                                                     std::cref(search_image),
                                                     std::ref(result_mat_second));

  // match async
  std::future<MatchResult> future_match_third = std::async(std::launch::async,
                                                     &Matcher::performMatchingOnSubset,
                                                     this,
                                                     std::cref(rotations),
                                                     split_one,
                                                     split_two,
                                                     std::cref(search_image),
                                                     std::ref(result_mat_third));

  // do second part of matching here
  std::vector<MatchResult> results(3);

  results[0] = performMatchingOnSubset(rotations, split_two, rotations->size(), search_image, result_mat);

  // read async results
  results[1] = future_match_second.get();
  results[2] = future_match_third.get();

  // compare
  const MatchResult& max = *std::max_element(results.begin(), results.end());

  match_location = max.best_loc;
  match_value = max.best_val;
  return (*rotations).at(max.index);
}

Matcher::MatchResult Matcher::performMatchingOnSubset(const PatchRotationContainerPtr &rotations,
                                                      std::size_t from, std::size_t to,
                                                      const cv::Mat &search_image,
                                                      cv::Mat& result_mat) const
{
  // return struct
  MatchResult result;
  result.best_val = -1.0;

  double minVal, maxVal;
  cv::Point minLoc, maxLoc;

  for (size_t i = from; i < to; i++)
  {
    matchTemplate(search_image, (*rotations)[i]->getPattern(), result_mat, CV_TM_CCORR_NORMED);

    // localizing the best match with minMaxLoc
    minMaxLoc(result_mat, &minVal, &maxVal, &minLoc, &maxLoc);

#ifdef KATANA_LT_MATCHER_MATCH_VALUES_DEBUG
    std::cout <<"MATCHER type: " <<(*rotations)[i]->getPatchType() <<" grad. " <<(*rotations)[i]->getStartDirection() <<" value " <<maxVal <<std::endl;
#endif

    // save best match
    if(maxVal > result.best_val)
    {
      result.best_val = maxVal;
      result.best_loc = maxLoc;
      result.index = i;
    }
  }

  return result;
}

TrackSegment Matcher::matchPatterns(const cv::Mat& input, const SearchTolerance& tolerance, const std::vector<PatchRotationContainerPtr>& patches)
{
  assert(!patches.empty() && "!patches.empty()");
#ifdef KATANA_LT_MATCHER
 cv::Mat searchSpc;
 input.copyTo(searchSpc);
#endif

  // try every patch
  double best_match_value = 0.0;
  cv::Point best_match_position;
  Patch::ConstPtr best_match;
  for (const PatchRotationContainerPtr& patch_rotations : patches)
  {
    double match_value;
    cv::Point match_position;
    Patch::ConstPtr match = matchPatchType(input, patch_rotations, tolerance, match_value, match_position);
    if (match != nullptr && match_value > best_match_value)
    {
      best_match = match;
      best_match_value = match_value;
      best_match_position = match_position;

    }
  }

  if (!best_match)
  {
    // return empty track with match value 0.0!
    return TrackSegment();
  }

  //<winner patch is in best_match>

  // build track information
  TrackSegment track = PatchFactory::getFactory()->getTrackByType(best_match->getPatchType(), best_match->getStartDirection());
  // bring track in searchspace's coordinate system
  track.translate(best_match_position);
  // set correct match value
  track.setMatchingValue(best_match_value);



  // re-construct searchspace for winner patch
  cv::RotatedRect searchspace = createSearchspaceForPatch(tolerance, best_match);


  cv::Point searchspaceCenterInSearchspace(searchspace.size.width / 2.0f, searchspace.size.height / 2.0f);
  cv::Point searchspaceCenterInInput(searchspace.center.x, searchspace.center.y);

  // rotate track back
  track.rotate(searchspaceCenterInSearchspace, searchspace.angle);
  // bring track in input's coordinate system
  track.translate(searchspaceCenterInInput - searchspaceCenterInSearchspace);

#ifdef KATANA_LT_MATCHER
  cv::Mat output;
  cv::Mat output2;
  input.copyTo(output2);
  cvtColor(output2, output, CV_GRAY2BGR);
  DrawUtil::drawTrack(output, track);
  DrawUtil::drawQuadrangle(output, m_field_of_vision_pixel);
  DrawUtil::writeImageToFile(output, "_" + oadrive::core::to_string(z++) +"_matched" );

  DrawUtil::drawSearchspace(searchSpc, searchspace);
  DrawUtil::writeImageToFile(searchSpc, "_" + oadrive::core::to_string(z) + "_searchspace");
#endif

  return track;
}


cv::Mat Matcher::cutOut(const cv::Mat& input, const cv::RotatedRect& searchspace) const
{
  cv::Mat result;
  // now rotate input image, getRotationMatrix2D assumes ccw rotation
  cv::Mat M = getRotationMatrix2D(searchspace.center, searchspace.angle, 1.0);
  cv::Mat rotated;
  warpAffine(input, rotated, M, input.size(), cv::INTER_CUBIC);

  // cut out the rectangle in the rotated image and put it to
  getRectSubPix(rotated, searchspace.size, searchspace.center, result);

  return result;
}

cv::RotatedRect Matcher::createSearchspaceForPatch(const SearchTolerance& tolerance, Patch::ConstPtr patch) const
{
  Pose2d searchspace_center_pose = tolerance.search_pose * patch->getCenterPose();

  const cv::Size2f size(tolerance.width + patch->getSize().width, tolerance.height + patch->getSize().height);

  return cv::RotatedRect(cv::Point2f(searchspace_center_pose.translation().x(), searchspace_center_pose.translation().y()),
                         size,
                         PoseTraits<Pose2d>::yaw(searchspace_center_pose) * 180/M_PI);
}

void Matcher::initializeTrapezoid(const ManagedPose::FieldOfViewPoints& field_of_view_points, float scaling_factor)
{
  assert(scaling_factor > 0);

  // Pixel coordinates for every trapezoid point
  for (size_t i = 0; i < 4; i++)
  {
    m_field_of_vision_pixel[i] = ManagedPose::transformCarToPixel(field_of_view_points[i]).getPose().getPose().translation();
  }

  // center of trapezoid for scaling
  const Position2d center_top = (m_field_of_vision_pixel[0] + m_field_of_vision_pixel[1]) * 0.5;
  const Position2d center_bottom = (m_field_of_vision_pixel[2] + m_field_of_vision_pixel[3]) * 0.5;

  m_field_of_view_center = (center_top + center_bottom) * 0.5;

  // scale every point towards the center of the trapezoid
  for (size_t i = 0; i < 4; i++)
  {
    m_field_of_vision_pixel[i] -= (1-scaling_factor)*(m_field_of_vision_pixel[i]-m_field_of_view_center).eval();
  }

}

bool Matcher::overlapping(const Quadrangle& q1, const Quadrangle& q2)
{
  // check if one point of q1 is within q2 and vice versa
  for (std::size_t i = 0; i < 4; i++)
  {
    if (isWithinOrOnLineOfQuadrangle(q1[i], q2))
      return true;

    if (isWithinOrOnLineOfQuadrangle(q2[i], q1))
      return true;
  }

  return false;
}

bool Matcher::isWithinOrOnLineOfQuadrangle(const Position2d& point, const Quadrangle& q)
{
  // check if the point lies right to every line of the trapezium ABCD: AB, BC, CD, DA
  // where ABCD = up_left, up_right, down_right, down_left

  for (size_t i = 0; i < 4; i++)
  {
    Eigen::Matrix<double, 2, 2> m;
    m.col(0) <<q[(i+1) % 4] - q[i];
    m.col(1) <<point - q[i];

    // If the determinant is negative, the point lies on the left hand to the line
    // and is therefore on the outside of the trapezium (assumed right-handed coordinate system)
    if (m.determinant() < 0.0)
      return false;
  }

  return true;
}

bool Matcher::isSearchspaceValid(const cv::RotatedRect& searchspace, const cv::Size& input_picture_size) const
{
  // fast-lane: check for center point
  Position2d center(searchspace.center.x, searchspace.center.y);
  if (!isWithinOrOnLineOfQuadrangle(center, m_field_of_vision_pixel))   //< center no in field of view, check for corner points
  {
    // get access to rectangle corner points, create Quadrangle
    cv::Point2f corners[4];
    searchspace.points(corners);

    Quadrangle s;
    for (std::size_t i = 0; i < 4; i++)
    {
      s[i] = Position2d(corners[i].x, corners[i].y);
    }

    // searchspace is does not overlap with field of vision, do not use...
    if (!overlapping(s, m_field_of_vision_pixel))
      return false;
  }

  return true;
}

bool Matcher::isSearchspaceInInput(const cv::RotatedRect& searchspace, const cv::Size& input_picture_size) const
{
  // check if searchspace is withing input picture size (this should in every case be true, otherwise the selected border is to small)
  const cv::Rect& b = searchspace.boundingRect();
  if (b.x < 0 || b.y < 0)
  {
    //assert(false && "Searchspace not within input picture. Maybe you need to increase the border?");
    return false;
  }
  if (b.x + b.width >= input_picture_size.width || b.y + b.height >= input_picture_size.height)
  {
    //assert(false && "Searchspace not within input picture. Maybe you need to increase the border?");
    return false;
  }

  return true;
}

void Matcher::clipSearchspace(cv::RotatedRect& searchspace, const cv::Size& input_picture_size) const
{
  // clip searchspace at input borders
  const cv::Rect& b = searchspace.boundingRect();
  if (b.x < 0)
  {
    // diff is negative!
    float diff = b.x;

    // shift center to the right
    searchspace.center -= cv::Point2f(diff/2.0, 0);

    // decrease width
    searchspace.size.width += diff;
  }
  if (b.y < 0)
  {
    // diff is negative!
    float diff = b.y;

    // shift center to the bottom
    searchspace.center -= cv::Point2f(0, diff/2.0);

    // decrease height
    searchspace.size.height += diff;
  }
  if (b.x + b.width >= input_picture_size.width)
  {
    // diff is negative!
    float diff = input_picture_size.width - (b.x + b.width);

    // shift center to the left
    searchspace.center += cv::Point2f(diff/2.0, 0);

    // decrease width
    searchspace.size.width += diff;
  }
  if (b.y + b.height >= input_picture_size.height)
  {
    // diff is negative!
    float diff = input_picture_size.height - (b.y + b.height);

    // shift center to the top
    searchspace.center += cv::Point2f(0, diff/2.0);

    // decrease height
    searchspace.size.height += diff;
  }
}

void Matcher::createPatchMask(const ManagedPose::FieldOfViewPoints &field_of_view_points)
{
  // Pixel coordinates for every trapezoid point
  for (size_t i = 0; i < 4; i++)
  {
    m_actual_field_of_vision[i] = ManagedPose::transformCarToPixel(field_of_view_points[i]).getPose().getPose().translation();
  }
}

void Matcher::drawMatchMask(cv::Mat& mask, const cv::RotatedRect &searchspace) const
{
  mask.setTo(cv::Scalar(0));

  // obtain searchspace base pose
  Pose2d searchspace_center;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(searchspace_center, searchspace.center.x, searchspace.center.y, searchspace.angle);

  Pose2d searchspace_center_to_base;
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(searchspace_center_to_base, -searchspace.size.width*0.5, -searchspace.size.height*0.5, 0.0);

  Pose2d searchspace_base_transf = (searchspace_center * searchspace_center_to_base).inverse();

  Position2d tmp;
   cv::Point point_draw_poly[1][4];

  // Pixel coordinates for every trapezoid point
  for (size_t i = 0; i < 4; i++)
  {
    tmp = searchspace_base_transf * m_actual_field_of_vision[i];

    point_draw_poly[0][i].x = tmp.x();
    point_draw_poly[0][i].y = tmp.y();
  }

  const cv::Point* ppt[1] = { point_draw_poly[0] };
  int npt[] = { 4 };
  cv::fillPoly(mask, ppt, npt, 1, cv::Scalar(255), int(8));

  // debug
  //DrawUtil::writeImageToFile(m_patch_mask, "_mask_");
}

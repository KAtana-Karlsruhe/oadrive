#include "RoadSignDetector.h"
#include <iostream>
#include <oadrive_vision/DrawUtil.h>
#include <iostream>

namespace oadrive {
namespace markers {

// we are searching for markers in the left part in the middle of the image
const cv::Rect RoadSignDetector::REGION_OF_INTEREST = cv::Rect(0, 140, 400, 200);


// allow markers that have these min and max sizes in pixel
const float RoadSignDetector::MARKER_MIN_SIZE_PIXEL = 12.0f;
const float RoadSignDetector::MARKER_MAX_SIZE_PIXEL = 64.0f;

// convert pixel sizes to fraction of REGION_OF_INTEREST max dimension
const float RoadSignDetector::MARKER_MIN_SIZE_FRACTION = MARKER_MIN_SIZE_PIXEL / std::max(REGION_OF_INTEREST.width, REGION_OF_INTEREST.height);
const float RoadSignDetector::MARKER_MAX_SIZE_FRACTION = MARKER_MAX_SIZE_PIXEL / std::max(REGION_OF_INTEREST.width, REGION_OF_INTEREST.height);



RoadSignDetector::RoadSignDetector():
  m_is_initialized(false)
{

}

RoadSignsContainer RoadSignDetector::detectRoadSigns(const cv::Mat &input)
{
  assert(m_is_initialized && "m_is_initialized");
  RoadSignsContainer markers;

  cv::Mat croppedReference(input, REGION_OF_INTEREST);


  cv::Mat cropped;
  // Copy the data into new matrix
  croppedReference.copyTo(cropped);

  // throws exception!
  m_marker_detector.detect(cropped, markers, cv::Mat(), cv::Mat(), m_marker_size, m_y_axis_is_perpendicular);

  #ifdef MARKER_DEBUG_FILES
    cv::Mat output;
    input.copyTo(output);
    for (const aruco::Marker& m : markers)
    {
      drawMarker(output, m);
    }
    drawRegionOfInterest(output);
    DrawUtil::writeImageToFile(output, "_MarkerDetector");
  #endif

  return markers;
}

bool RoadSignDetector::initializeWithoutCalibration(const double markerSize, aruco::Dictionary dictionary)
{
  if (!m_is_initialized)
  {
    m_marker_size = markerSize;

    if (!aruco::HighlyReliableMarkers::loadDictionary(dictionary))
    {
      std::cout << "Could not initialize RoadSignDetector" << endl;
      return false;
    }
    m_dictionary = dictionary;
    m_marker_detector.setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);

    m_marker_detector.setThresholdParams(21, 7);
    m_marker_detector.setDesiredSpeed(3);

    try
    {
      m_marker_detector.setMinMaxSize(MARKER_MIN_SIZE_FRACTION, MARKER_MAX_SIZE_FRACTION);
    }
    catch (...)
    {
      std::cout << "Could not initialize RoadSignDetector" << endl;
      return false;
    }

    m_is_initialized = true;

    return true;
  }
  return false;
}

void RoadSignDetector::drawMarker(cv::Mat &output, const aruco::Marker &marker)
{
  cv::Mat croppedReference(output, REGION_OF_INTEREST);
  marker.draw(croppedReference, cv::Scalar(0, 0, 255), 1);
  cv::Point2f center = marker.getCenter();

  float area = marker.getArea();
  DrawUtil::putTextAt(croppedReference, center, std::to_string(area));

  float aspect = calculateAspectRatio(marker);
  cv::Point bottom(center.x + 50, center.y + 50);
  DrawUtil::putTextAt(croppedReference, bottom, std::to_string(aspect));
}

void RoadSignDetector::drawRegionOfInterest(cv::Mat &output)
{
  cv::rectangle(output, REGION_OF_INTEREST, cv::Scalar(255, 0, 255), 3);
}

float RoadSignDetector::calculateAspectRatio(const aruco::Marker &marker)
{
  //  0 1
  //  3 2

  Quadrangle marker_points;
  for (u_int8_t i = 0; i < 4; i++)
  {
    marker_points[i].x() = marker[i].x;
    marker_points[i].y() = marker[i].y;
  }


  // return 0-1 / 3-2
  return ((marker_points[0] - marker_points[1]).norm() + (marker_points[2] - marker_points[3]).norm()) /
      ((marker_points[0] - marker_points[3]).norm() + (marker_points[1] - marker_points[2]).norm());
}

bool RoadSignDetector::isUpsideDown(const aruco::Marker& marker, const int margin)
{
  // if 0. corner is farther down than 3.corner - margin than we are upside down
  return marker[0].y >= marker[3].y - margin;
}

} // end of ns
} // end of ns

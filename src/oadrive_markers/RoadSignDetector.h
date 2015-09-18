#ifndef ROADSIGNDETECTOR_H
#define ROADSIGNDETECTOR_H


#include <vector>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <aruco/aruco.h>
#include <aruco/highlyreliablemarkers.h>

//#define MARKER_DEBUG_FILES

namespace oadrive {
namespace markers {
typedef vector<aruco::Marker> RoadSignsContainer;

class RoadSignDetector
{
public:
  //! default constructor
  RoadSignDetector();

  //!
  //! \brief detectRoadSigns runs aruco detection on input matrix
  //! \param input input image to be searched for markers
  //! \return detected road signs with NO 3D information!
  //!
  RoadSignsContainer detectRoadSigns(const cv::Mat& input);

  //!
  //! \brief initializeWithoutCalibration initializes RoadSignDetector without intrinsic camera parameters
  //! \param markerSize markerSize in meter
  //! \param dictionary dictionary that contains information about board of used markers and ids
  //! \return false if initialization failed, true if already initialized or initialization was successful
  bool initializeWithoutCalibration(const double markerSize, aruco::Dictionary dictionary);

  static void drawMarker(cv::Mat& output, const aruco::Marker& marker);
  static void drawRegionOfInterest(cv::Mat& output);

  //! Calculates yaw respective to camera
  static float calculateAspectRatio(const aruco::Marker& marker);

  static bool isUpsideDown(const aruco::Marker& marker, const int margin);


  static const cv::Rect REGION_OF_INTEREST;
  static const float MARKER_MIN_SIZE_PIXEL;
  static const float MARKER_MAX_SIZE_PIXEL;
  static const float MARKER_MIN_SIZE_FRACTION;
  static const float MARKER_MAX_SIZE_FRACTION;

private:
  /************member needed for aruco marker detection********************/
  //! worker to detect markers in input image
  aruco::MarkerDetector m_marker_detector;
  //! marker size in meters
  double m_marker_size;
  //! if y-axis is perpendicular to ground surface
  bool m_y_axis_is_perpendicular;
  //! highlyreliable dictionary
  aruco::Dictionary m_dictionary;

  /********************************own members*****************************/
  //! used to only initialize once
  bool m_is_initialized;
};

} // end of ns
} // end of ns

#endif // ROADSIGNDETECTOR_H

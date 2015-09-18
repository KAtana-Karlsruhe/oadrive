#ifndef OADRIVE_VISION_DRAWUTIL_H_INCLUDED
#define OADRIVE_VISION_DRAWUTIL_H_INCLUDED

#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <ctime>

#include <oadrive_vision/TrackSegment.h>
#include <oadrive_vision/Matcher.h>

class DrawUtil
{
public:
  DrawUtil();

  static const cv::Scalar DARKRED;
  static const cv::Scalar DARKBLUE;
  static const cv::Scalar DARKGREEN;
  static const cv::Scalar GOLD;
  static const cv::Scalar MAGENTA;

  static const cv::Scalar YELLOW;
  static const cv::Scalar ORANGE;
  static const cv::Scalar RED;

  static const cv::Scalar GREEN;
  static const cv::Scalar SEAGREEN;
  static const cv::Scalar CYAN;

  static const cv::Scalar GRAY;

  static const int COLOR_COUNT;
  static void drawSearchspace(cv::Mat &input, cv::RotatedRect searchspace, int index = 0);

  static void drawPoint(cv::Mat &input, cv::Point position, int id = 0);

  static void drawManagedPose(cv::Mat &input, ManagedPose point);

  static void drawTrack(cv::Mat &input, TrackSegment track, int id = 0);
  static void drawSearchTolerance(cv::Mat &input, const Matcher::SearchTolerance& tolerance, int index = 0);

  static void drawQuadrangle(cv::Mat &input, std::array<oadrive::core::Position2d, 4> corners);

  static void putTextAt(cv::Mat& input, const cv::Point& position, const std::string& text, int color = 0);

  static void writeImageToFile(const cv::Mat& image, const std::string& suffix, bool addTimestamp = true);
};

#endif

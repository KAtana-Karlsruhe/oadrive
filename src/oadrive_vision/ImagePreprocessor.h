#ifndef OADRIVE_VISION_IMAGEPREPROCESSOR_H_INCLUDED
#define OADRIVE_VISION_IMAGEPREPROCESSOR_H_INCLUDED

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <array>


#include "ManagedPose.h"

class ImagePreprocessor {
public:
  //!
  //! \brief ImagePreprocessor
  //!
  ImagePreprocessor() = delete;

  //! Destructor
  virtual ~ImagePreprocessor();

  //!
  //! \brief ImagePreprocessor
  //! \param thresholdValue thresholdValue for binary thresholding operation
  //!
  ImagePreprocessor(int thresholdValue);

  //!
  //! \brief in-/ouput pixel coordinates for IPM
  //!
  typedef std::vector<cv::Point2f> IpmCalibrationInputPoints;
  typedef std::vector<cv::Point2f> IpmCalibrationOutputPoints;

  //! Load ipm calibration from file and create IPM
  bool initialize( const std::string& filename);

  //!
  //! \brief convertToBirdView
  //! \param input matrix in BGR color space
  //! \param output output variable with the matrix in BGR color space that represents a bird view image of the input
  //!
  void convertToBirdView(cv::Mat &input, cv::Mat &output);

  //!
  //! \brief doBinaryThreshhold
  //! \param input matrix in BGR color space
  //! \param output matrix in GRAY color space with different dimensions than input
  //! \param thresholdValue all pixels with an intensity higher than this in input will be white, all others black in output
  //!
  void doBinaryThreshhold(cv::Mat &input, cv::Mat &output);

  //!
  //! \brief doBinaryThreshhold
  //! \param input matrix in BGR color space
  //! \param output matrix in GRAY color space with different dimensions than input
  //!
  void doHSVThreshold(cv::Mat &input, cv::Mat &output, bool extended);

  cv::Mat filterHSV(cv::Mat input);
  cv::Mat filterRGB(cv::Mat input, int thresh);
  cv::Mat filterCanny(cv::Mat input, int thresh);
  cv::Mat fusion(cv::Mat source1, cv::Mat source2, cv::Mat source3);
  int estimateThreshold(cv::Mat &image, double percentage);

  int getThresholdValue() const { return m_thresholdValue; }
  void setThresholdValue(int thresholdValue) { m_thresholdValue = thresholdValue; }

private:
  //! create IPM from loaded calibration data
  void createIPM();

  //!
  //! \brief loadIpmCalibration: loads ipm calibration point pairs from config XML
  //! \param XML file, array to write points to
  //!
  bool loadIpmCalibration(const std::string& filename, IpmCalibrationInputPoints& input_points,
                          IpmCalibrationOutputPoints& output_points, cv::Size& output_image_size_pixel) const;

  //! ********* XML params ********
  static const constexpr char* XML_NODE_TOP = "ipm_calibration";
  static const constexpr char* XML_NODE_PAIR = "pair";
  static const constexpr char* XML_NODE_SIZE = "size";
  static const constexpr char* XML_NODE_SIZE_WIDTH = "width";
  static const constexpr char* XML_NODE_SIZE_HEIGHT = "height";
  static const constexpr char* XML_NODE_PAIR_ID = "id";
  static const constexpr char* XML_ELEMENT_POINT_IN = "camera_in";
  static const constexpr char* XML_VALUE_X = "xvalue";
  static const constexpr char* XML_VALUE_Y = "yvalue";
  static const constexpr char* XML_ELEMENT_POINT_OUT = "camera_out";

  static const char* XML_PAIR_IDS[];
  enum class CALIBRATION_PAIR : u_int8_t {
    UP_LEFT,
    UP_RIGHT,
    DOWN_RIGHT,
    DOWN_LEFT,
    CALIBRATION_PAIR_COUNT = 4
  };

  //! IPM
  IPM* m_IPM;

  //! Threshold
  int m_thresholdValue;

  //! Border height at image bottom (in px)
  cv::Point m_bottom_border_height;

};

#endif

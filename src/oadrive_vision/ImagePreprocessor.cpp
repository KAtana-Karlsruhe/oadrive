#include "ImagePreprocessor.h"

using namespace cv;

#include <tinyxml/tinyxml.h>
#include <oadrive_vision/DrawUtil.h>

const char* ImagePreprocessor::XML_PAIR_IDS[] =  // same order as CALIBRATION_PAIR enum!
{
  "up-left",
  "up-right",
  "down-right",
  "down-left",
};


ImagePreprocessor::ImagePreprocessor(int thresholdValue)
  : m_IPM(nullptr)
  , m_thresholdValue(thresholdValue)
{
}

ImagePreprocessor::~ImagePreprocessor()
{
  if (m_IPM != nullptr)
    delete m_IPM;
}

bool ImagePreprocessor::initialize(const std::string& filename)
{
  // try to initialize managed pose
  if (!ManagedPose::readTransformationParameters(filename))
    return false;

  IpmCalibrationInputPoints input;      //< this is in PIXEL
  IpmCalibrationOutputPoints output;    //< this is in cm

  cv::Size output_image_size;
  cv::Size input_image_size(640, 480);  //< fixed camera resolution

  // load output positions from XML to destination pixel coordinates
  if (!loadIpmCalibration(filename, input, output, output_image_size))
    return false;

  // convert to pixel
  for (IpmCalibrationOutputPoints::iterator it = output.begin(); it != output.end(); it++)
  {
    ManagedPose tmp(it->x, it->y, 0, ManagedPose::Context::IN_CAR);
    tmp = ManagedPose::transformCarToPixel(tmp);
    it->x = tmp.getX();
    it->y = tmp.getY();
  }

  // create IPM
  m_IPM = new IPM(input_image_size, output_image_size, input, output);

  // calculate height of border
  oadrive::core::Position2d start_field_of_view = ManagedPose::transformCarToPixel(ManagedPose(5000, 0, 0, ManagedPose::IN_CAR)).getPose().getPose().translation();
  m_bottom_border_height.x = output_image_size.width - 1;
  m_bottom_border_height.y = start_field_of_view.y();

  return true;
}

void ImagePreprocessor::convertToBirdView(Mat &input, Mat &output)
{
  assert(m_IPM != nullptr && "You need to initialize the ImagePreprocessor");

  m_IPM->applyHomography(input, output);

  // draw border at bottom of image black (IPM (correctly) transforms some of the image at this place)
  cv::rectangle(output, cv::Point(0, output.size().height - 1), m_bottom_border_height, cv::Scalar(0), CV_FILLED);
}

void ImagePreprocessor::doBinaryThreshhold(Mat &input, Mat &output)
{
  Mat temp;
  cvtColor(input, temp, CV_BGR2GRAY);
  cv::threshold(temp, output, m_thresholdValue, 255, cv::THRESH_BINARY);
}

void ImagePreprocessor::doHSVThreshold(Mat &input, Mat &output, bool extended)
{
    if(extended) {
        Mat hsv = filterHSV(input);

        //prepare mask
        Mat mask;
        cv::normalize(hsv, mask, 0., 1., CV_MINMAX);
        cvtColor(mask , mask, CV_GRAY2BGR);
        Mat rgbMask;
        rgbMask = input.mul(mask);
        cvtColor(rgbMask, rgbMask, CV_BGR2GRAY);

        Scalar mean = cv::mean(rgbMask);
        int average = (mean.val[0]*input.size().area())/cv::countNonZero(rgbMask); //COULD make sense as threshold for rgb and canny filtering,

        Mat rgb = filterRGB(rgbMask, 200);
        Mat canny = filterCanny(input, 200);
        output = fusion(hsv, rgb, canny);
    } else {
        output = filterHSV(input);
    }
}

Mat ImagePreprocessor::filterHSV(Mat input) {
    Mat im_hsv;
    Mat enhanced;
    cvtColor(input ,im_hsv,CV_BGR2HSV);
    cv::inRange(im_hsv, Scalar(0, 0, 255*0.55), Scalar(360/2, 255*0.55, 255), enhanced);
    //cv::inRange(im_hsv, Scalar(0, 0, 255*0.65), Scalar(360/2, 255*0.50, 255), enhanced);

    return enhanced;
}

Mat ImagePreprocessor::filterRGB(Mat input, int thresh) {
    Mat enhanced;
    cv::threshold(input, enhanced, thresh*0.70, 255, CV_THRESH_BINARY);

    return enhanced;
}

Mat ImagePreprocessor::filterCanny(Mat input, int thresh) {
    Mat canny;
    input.copyTo(canny);
    cv::threshold(canny, canny, thresh*0.7, 255, CV_THRESH_BINARY);
    blur( canny, canny, Size(3,3) );
    cv::Canny(canny, canny, 5, 3*5, 5, true);
    std::vector<std::vector<Point> > contours;
    findContours(canny, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    // clean the image
    canny.setTo(Scalar(0));

    for(unsigned int i = 0; i < contours.size(); i++) {
        // draw contour of leaf in image
        // Find the bounding rectangle for biggest contour
        if(boundingRect(contours[i]).area() > 35) {
            fillConvexPoly(canny, contours[i], Scalar(255), 8, 0);
        }
    }

    return canny;
}

Mat ImagePreprocessor::fusion(Mat source1, Mat source2, Mat source3) {
    int thresh = 255/3;
    Mat norm_source1, norm_source2, norm_source3, sum;
    cv::normalize(source1, norm_source1, 0., thresh, CV_MINMAX);
    cv::normalize(source2, norm_source2, 0., thresh, CV_MINMAX);
    cv::normalize(source3, norm_source3, 0., thresh, CV_MINMAX);

    sum = norm_source1 + norm_source2 + norm_source3;
    cv::threshold(sum, sum, 2*thresh-1, 255, CV_THRESH_BINARY);
    return sum;
}


int ImagePreprocessor::estimateThreshold(Mat &image, double percentage) {
    Mat grayScale;
    cvtColor(image, grayScale, CV_BGR2GRAY);
    Mat hist = Mat::zeros(256, 1, CV_32SC1);
    int histSize = 256;

    float range[] = { 0, 256 } ; //the upper boundary is exclusive
    const float* histRange = { range };
    cv::calcHist( &grayScale, 1, 0, Mat(), hist, 1, &histSize, &histRange, true, false );

    int neededPixels = grayScale.size().area()*percentage;
    int accu = 0;
    int threshold = 255;
    while(accu < neededPixels && threshold > 0) {
        accu += hist.at<float>(threshold, 0);
        threshold--;
    }

    return threshold;
}

bool ImagePreprocessor::loadIpmCalibration(const std::string& filename,
                                           IpmCalibrationInputPoints& input_points,
                                           IpmCalibrationOutputPoints& output_points,
                                           cv::Size& output_image_size_pixel) const
{
  TiXmlDocument doc(filename);
  if (!doc.LoadFile())
  {
    return false;
  }

  TiXmlElement* top_node = doc.FirstChildElement(XML_NODE_TOP);
  if (top_node == nullptr)
    return false;

  // iterate over pairs
  input_points.resize(4, cv::Point2f()); //< resize vector
  output_points.resize(4, cv::Point2f()); //< resize vector

  for (TiXmlElement* pair = top_node->FirstChildElement(XML_NODE_PAIR); pair != nullptr; pair = pair->NextSiblingElement(XML_NODE_PAIR))
  {
    std::string current_pair_id = pair->Attribute(XML_NODE_PAIR_ID);

    u_int8_t i;
    for (i = 0; i < (u_int8_t)CALIBRATION_PAIR::CALIBRATION_PAIR_COUNT; i++)
    {
      if (current_pair_id == XML_PAIR_IDS[i])
        break;
    }
    if (i == (u_int8_t)CALIBRATION_PAIR::CALIBRATION_PAIR_COUNT) //< error wrong pair id
      return false;

    // read pairs:
    TiXmlElement* point_in = pair->FirstChildElement(XML_ELEMENT_POINT_IN);
    if (point_in == nullptr)
      return false;

    if (point_in->QueryFloatAttribute(XML_VALUE_X, &input_points[i].x) != TIXML_SUCCESS)
      return false;
    if (point_in->QueryFloatAttribute(XML_VALUE_Y, &input_points[i].y) != TIXML_SUCCESS)
      return false;


    TiXmlElement* point_out = pair->FirstChildElement(XML_ELEMENT_POINT_OUT);
    if (point_out == nullptr)
      return false;

    if (point_out->QueryFloatAttribute(XML_VALUE_X, &output_points[i].x) != TIXML_SUCCESS)
      return false;
    if (point_out->QueryFloatAttribute(XML_VALUE_Y, &output_points[i].y) != TIXML_SUCCESS)
      return false;

  }

  // read size of output picture
  TiXmlElement* size = top_node->FirstChildElement(XML_NODE_SIZE);
  if (size == nullptr)
    return false;

  float size_width;   //< in 1mm/10
  float size_height;  //< in 1mm/10

  if (size->QueryFloatAttribute(XML_NODE_SIZE_WIDTH, &size_width) != TIXML_SUCCESS)
    return false;

  if (size->QueryFloatAttribute(XML_NODE_SIZE_HEIGHT, &size_height) != TIXML_SUCCESS)
    return false;

  // convert to pixel
  output_image_size_pixel.width = size_width * ManagedPose::getPixelRatio();
  output_image_size_pixel.height = size_height * ManagedPose::getPixelRatio();

  return true;

}

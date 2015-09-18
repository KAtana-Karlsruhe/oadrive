#include "Searchspace.h"

Searchspace::Searchspace()
{
}

Searchspace::Searchspace(cv::Mat &input, cv::Point center, cv::Size size, int rotation)
{
  initializeSearchspace(input, center, size, rotation);
}

void Searchspace::initializeSearchspace(cv::Mat &input, cv::Point center, cv::Size size, int rotation)
{
  int width = size.width;
  int height = size.height;

  // TODO: check if correct
  float angle = rotation;

  // construct searchspace as RotatedRect
  cv::RotatedRect tmp(center, cv::Size2f(width,height), angle);

  // make sure ROI is within boundaries of input matrix
  cv::Rect boundingRect = tmp.boundingRect();
  if (boundingRect.x < 0)
  {
    tmp.center = tmp.center - cv::Point2f(boundingRect.x, 0);
  }
  if (boundingRect.y < 0)
  {
    tmp.center = tmp.center - cv::Point2f(0, boundingRect.y);
  }
  if (boundingRect.x + boundingRect.width > input.cols)
  {
    tmp.center = tmp.center - cv::Point2f(boundingRect.x + boundingRect.width - input.cols, 0);
  }
  if (boundingRect.y + boundingRect.height > input.rows)
  {
    tmp.center = tmp.center - cv::Point2f(0, boundingRect.y + boundingRect.height - input.rows);
  }

  m_searchspaceRect = tmp;

  // now rotate input image
  cv::Mat M = getRotationMatrix2D(m_searchspaceRect.center, m_searchspaceRect.angle, 1.0);
  cv::Mat rotated;
  warpAffine(input, rotated, M, input.size(), cv::INTER_CUBIC);

  // cut out the rectangle in the rotated image and put it to m_image
  getRectSubPix(rotated, m_searchspaceRect.size, m_searchspaceRect.center, m_image);
  cvtColor(m_image, m_coloredImage, CV_GRAY2BGR);
}

cv::Point Searchspace::getPositionRelativeToInput() const
{
  cv::Point center = cv::Point(m_searchspaceRect.center.x, m_searchspaceRect.center.y);
  cv::Point half = cv::Point(m_searchspaceRect.size.width/2.0f, m_searchspaceRect.size.height/2.0f);
  cv::Point topLeft = center - half;
  return topLeft;
}

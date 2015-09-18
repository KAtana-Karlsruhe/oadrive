#ifndef OADRIVE_VISION_SEARCHSPACE_H_INCLUDED
#define OADRIVE_VISION_SEARCHSPACE_H_INCLUDED

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Searchspace
{
public:

  //! Default constructor
  Searchspace();

  /*! Builds a searchspace inside input matrix that fulfills parameters as best as possible
   * \param input input matrix to be searched for
   * \param center target center of searchspace in input's coordinates. final value might be different
   * \param searchspaceSize target searchspaceSize. final value might be different
   * \param rotation rotation in degree in ccw direction
   */
  Searchspace(cv::Mat &input, cv::Point center, cv::Size searchspaceSize, int rotation);

  //! \returns image information of searchspace
  cv::Mat getImage() const {return m_image;}

  //! \return returns colored image information of searchspace
  cv::Mat getColoredImage() const {return m_coloredImage;}

  //! \return returns the searchspace's rotation relative to the input matrix
  float getRotationRelativeToInput() const {return -m_searchspaceRect.angle;}

  //! \return returns the searchspace's position in input's coordinate system
  cv::Point getPositionRelativeToInput() const;

  cv::RotatedRect getAsRotatedRect() const {return m_searchspaceRect;}

private:

  //! contains pixel information of the searchspace in unrotated form
  cv::Mat m_image;


  //! contains pixel information in BGR
  cv::Mat m_coloredImage;


  //! contains position and rotation relative to the input matrix
  cv::RotatedRect m_searchspaceRect;


  /*! initialize searchspace for parameter docu see constructor
   * \param input
   * \param center
   * \param size
   * \param rotation
   */
  void initializeSearchspace(cv::Mat &input, cv::Point center, cv::Size size, int rotation);

};

#endif

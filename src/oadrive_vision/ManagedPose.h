#ifndef OADRIVE_VISION_MANAGEDPOSE_H_INCLUDED
#define OADRIVE_VISION_MANAGEDPOSE_H_INCLUDED

#include <iostream>
#include <string>
#include "IPM.h"
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <map>
#include <oadrive_core/ExtendedPose2d.h>

typedef std::array<oadrive::core::Position2d, 4> Quadrangle;

class ManagedPose
{
public:
  enum Context {IN_WORLD, IN_PIXEL, IN_CAR};
  static std::map<Context, std::string> ContextNames;

  //! field of view, used to check for valid searchspace
  typedef std::array<ManagedPose, 4> FieldOfViewPoints;


  //! Constructor
  ManagedPose();

  /*! \param x in context's coordinates
   *  \param y in context's coordinates
   *  \param theta in context's rotation sense in radian.
   *  \param context coordinate system information
   */
  ManagedPose(float x, float y, float theta, Context context);


  /*! \param poseInContext pose in context's coordinates
   *  \param context coordinate system information
   */
  ManagedPose(oadrive::core::ExtendedPose2d poseInContext, Context context);

  //! \return returns current context
  Context getContext() const {return m_context;}

  //! \return oadrive::core::ExtendedPose2d of this ManagedPose
  const oadrive::core::ExtendedPose2d& getPose() const {return m_poseInContext;}

  //! \return the theta of the ManagedPose in Degree.
  float getThetaInDegree() const;

  //! \return the theta of the ManagedPose in Rad.
  float getThetaInRad() const;

  //! \return the X value of the ManagedPose.
  float getX() const;

  //! \return the Y value of the ManagedPose.
  float getY() const;

  /*! \param center the x and y value of center describe the roation-center and the theta of center is the roation-angle.
   * \return the rotated ManagedPose.
   */
  ManagedPose rotate(ManagedPose center);

  std::string toString();

  ManagedPose operator*(float rhs) const
  {
    oadrive::core::ExtendedPose2d pose;
    pose.setX(getPose().getX()*rhs);
    pose.setY(getPose().getY()*rhs);
    pose.setYaw(getPose().getYaw());
    return ManagedPose(pose, getContext());
  }

  ManagedPose operator+(const ManagedPose& rhs) const
  {
    if(getContext() != rhs.getContext())
    {
      std::cout << "Contexts do NOT match (+)." << std::endl;
      throw 1;
    }

    return ManagedPose(getPose().getX() + rhs.getPose().getX(), getPose().getY() + rhs.getPose().getY(), (getPose().getYaw() + rhs.getPose().getYaw()), getContext());
  }

  ManagedPose& operator+=(const ManagedPose& rhs)
  {
    if(getContext() != rhs.getContext())
    {
      std::cout << "Contexts do NOT match (+=)." << std::endl;
      throw 1;
    }
    oadrive::core::ExtendedPose2d pose;
    pose.setX(getPose().getX() + rhs.getPose().getX());
    pose.setY(getPose().getY() + rhs.getPose().getY());
    pose.setYaw(getPose().getYaw() + rhs.getPose().getYaw());
    this->m_poseInContext = pose;
    return *this;
  }


  ManagedPose operator-(const ManagedPose& rhs) const
  {
    if(getContext() != rhs.getContext())
    {
      std::cout << "Contexts do NOT match (-)." << std::endl;
      throw 1;
    }
    return ManagedPose(getPose().getX() - rhs.getPose().getX(), getPose().getY() - rhs.getPose().getY(), (getPose().getYaw() - rhs.getPose().getYaw()), getContext());
  }

  ManagedPose& operator-=(const ManagedPose& rhs)
  {
    if(getContext() != rhs.getContext())
    {
      std::cout << "Context do NOT match (-=)." << std::endl;
      throw 1;
    }
    oadrive::core::ExtendedPose2d pose;
    pose.setX(getPose().getX() - rhs.getPose().getX());
    pose.setY(getPose().getY() - rhs.getPose().getY());
    pose.setYaw(getPose().getYaw() - rhs.getPose().getYaw());
    this->m_poseInContext = pose;
    return *this;
  }

  ManagedPose& operator=(const ManagedPose& rhs)
  {
    this->m_poseInContext = rhs.getPose();
    this->m_context = rhs.getContext();

    return *this;
  }

  //! \return returns pixelToCentimeterRatio in top view image
  static float getPixelRatio() {return static_pixel_ratio;}


  //static ManagedPose transformWorldToPixel(ManagedPose poseInWorld, ManagedPose vehicle_pose);
  //static ManagedPose transformWorldToCar(ManagedPose poseInWorld, ManagedPose vehicle_pose);

  //static ManagedPose transformCarToWorld(ManagedPose poseInCar, ManagedPose carPose);
  static ManagedPose transformCarToPixel(const ManagedPose& poseInCar);

  //static ManagedPose transformPixelToWorld(ManagedPose poseInPixel, ManagedPose carPose);
  static ManagedPose transformPixelToCar(const ManagedPose& poseInPixel);

  static const cv::Point2d& getPixelCoordinateOrigin()  { return static_pixel_coordinate_origin; }

  /**
   * @brief readTransformationParameters
   *        fill in m_pixel_ratio and m_pixel_coordinate_origin constants
   * @param filename
   * @return true on success, false otherwise
   */
  static bool readTransformationParameters(const std::string& filename);

  static const cv::Rect& getFieldOfVisionRectanglePixel()     { return m_field_of_vision_rectangle_pixel; }

private:
  oadrive::core::ExtendedPose2d m_poseInContext;
  Context m_context;


  //! Coordinate origin of pixel coordinate system in vehicle coordinates (read from xml)
  static cv::Point2d static_pixel_coordinate_origin;

  //! Pixel to cm ratio (read from xml)
  static double static_pixel_ratio;

  //! flag if initialized
  static bool static_initialized;

  //! Actaul Field of vision as rectangle for IPM adaptive threshold
  static const ManagedPose FIELD_OF_VISION_RECTANGLE_POS;
  static const ManagedPose FIELD_OF_VISION_RECTANGLE_SIZE;
  //! Field of vision rectangle, pixel coordinates
  static cv::Rect m_field_of_vision_rectangle_pixel;

  //! ********* XML params ********
  static const constexpr char* XML_NODE_TOP = "pixel_conversion";
  static const constexpr char* XML_ELEMENT_RATIO = "pixelRatio";
  static const constexpr char* XML_ELEMENT_RATIO_VALUE = "value";
  static const constexpr char* XML_ELEMENT_ORIGIN = "pixel_coordinate_origin";
  static const constexpr char* XML_ELEMENT_ORIGIN_XVALUE = "xvalue";
  static const constexpr char* XML_ELEMENT_ORIGIN_YVALUE = "yvalue";
};

#endif

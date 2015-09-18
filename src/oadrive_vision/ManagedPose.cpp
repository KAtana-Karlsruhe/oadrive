#include "ManagedPose.h"

#include <tinyxml/tinyxml.h>
#include <oadrive_core/Utility.h>

using namespace std;

// create static variables
cv::Point2d ManagedPose::static_pixel_coordinate_origin = cv::Point2d(285.0f, 277.67f);
double ManagedPose::static_pixel_ratio = 0.03f;
bool ManagedPose::static_initialized = false;

cv::Rect ManagedPose::m_field_of_vision_rectangle_pixel;

map<ManagedPose::Context, string> createContextNameMap()
{
  map<ManagedPose::Context, string> m;
  m[ManagedPose::IN_CAR]  = "IN_CAR";
  m[ManagedPose::IN_PIXEL]= "IN_PIXEL";
  m[ManagedPose::IN_WORLD]= "IN_WORLD";
  return m;
}

map<ManagedPose::Context, string> ManagedPose::ContextNames = createContextNameMap();

const ManagedPose ManagedPose::FIELD_OF_VISION_RECTANGLE_POS = ManagedPose(22000, 7500, 0, ManagedPose::Context::IN_CAR);
const ManagedPose ManagedPose::FIELD_OF_VISION_RECTANGLE_SIZE = ManagedPose(11500, 11500, 0, ManagedPose::Context::IN_CAR);

ManagedPose::ManagedPose()
{
}

ManagedPose::ManagedPose(float x, float y, float theta, Context context)
{
  m_poseInContext.setX(x);
  m_poseInContext.setY(y);
  m_poseInContext.setYaw(theta);
  m_context = context;
}

float ManagedPose::getThetaInDegree() const
{
  return m_poseInContext.getYaw()*180/M_PI;
}

float ManagedPose::getThetaInRad() const
{
  return m_poseInContext.getYaw();
}

float ManagedPose::getX() const
{
  return m_poseInContext.getX();
}

float ManagedPose::getY() const
{
  return m_poseInContext.getY();
}

ManagedPose::ManagedPose(oadrive::core::ExtendedPose2d poseInContext, Context context)
{
  m_poseInContext = poseInContext;
  m_context = context;
}

ManagedPose ManagedPose::rotate(ManagedPose center)
{
  if (center.getContext() != getContext())
  {
    cout << "pose has different Context than current pose (rotate)" << endl;
    throw 1;
  }


  // compute in Point2f to reduce computation errors
  cv::Point2f rotated(getPose().getX(), getPose().getY());

  float rotation = center.getPose().getYaw();
  // rotate point by rotationInRad around center
  if (rotation != 0)
  {
    // first, translate the point
    cv::Point2f translated = cv::Point2f(getPose().getX(), getPose().getY()) - cv::Point2f(center.getPose().getX(), center.getPose().getY());

    // rotate it as usual
    rotated.x = translated.x * std::cos(rotation) - translated.y * std::sin(rotation);
    rotated.y = translated.x * std::sin(rotation) + translated.y * std::cos(rotation);

    // translate back
    rotated += cv::Point2f(center.getPose().getX(), center.getPose().getY());
  }

  ManagedPose result(rotated.x, rotated.y, (getPose().getYaw() + center.getPose().getYaw()), getContext());
  return result;

}

/*
ManagedPose ManagedPose::transformWorldToPixel(ManagedPose poseInWorld, ManagedPose vehicle_pose)
{
  assert(poseInWorld.getContext() == IN_WORLD && "poseInWorld not in Context IN_WORLD!");
  assert(vehicle_pose.getContext() == IN_WORLD && "vehicle_pose not in Context IN_WORLD!");

  // obtain pose in car coordinates
  ManagedPose poseInCar = ManagedPose::transformWorldToCar(poseInWorld, vehicle_pose);

  // obtain pose in pixel coordinates
  ManagedPose poseInPixel = ManagedPose::transformCarToPixel(poseInCar);

  return poseInPixel;
}

ManagedPose ManagedPose::transformWorldToCar(ManagedPose poseInWorld, ManagedPose vehicle_pose)
{
  assert(poseInWorld.getContext() == IN_WORLD && "poseInWorld not in Context IN_WORLD!");
  assert(vehicle_pose.getContext() == IN_WORLD && "vehicle_pose not in Context IN_WORLD!");


  // use odometry information of vehicle_pose to obtain world coordinates
  oadrive::core::ExtendedPose2d pose_in_car;
  pose_in_car.pose() = vehicle_pose.getPose().pose().inverse() * poseInWorld.getPose().pose();
  ManagedPose managed_pose_in_car(pose_in_car, IN_CAR);
  return managed_pose_in_car;
}*/

ManagedPose ManagedPose::transformPixelToCar(const ManagedPose& poseInPixel)
{
  assert(poseInPixel.getContext() == IN_PIXEL && "poseInPixel not in Context IN_PIXEL!");

  // scale to 0.1mm
  oadrive::core::Pose2d pose_scaled = poseInPixel.getPose().getPose();
  pose_scaled.translation() /= getPixelRatio();


  oadrive::core::Pose2d pose_car;
  oadrive::core::PoseTraits<oadrive::core::Pose2d>::fromPositionAndOrientationRPY(pose_car,
                    static_pixel_coordinate_origin.x - pose_scaled.translation().y() ,
                    static_pixel_coordinate_origin.y - pose_scaled.translation().x(),
                    -oadrive::core::PoseTraits<oadrive::core::Pose2d>::yaw(pose_scaled)     //< rotation flipped
                                                                                  );
  return ManagedPose(oadrive::core::ExtendedPose2d(pose_car), IN_CAR);
}

ManagedPose ManagedPose::transformCarToPixel(const ManagedPose& poseInCar)
{
  assert(poseInCar.getContext() == IN_CAR && "poseInCar not in Context IN_CAR!");

  oadrive::core::Pose2d pixel_coord;
  oadrive::core::PoseTraits<oadrive::core::Pose2d>::fromPositionAndOrientationRPY(pixel_coord,
                    static_pixel_coordinate_origin.y - poseInCar.getPose().getY() ,
                    static_pixel_coordinate_origin.x - poseInCar.getPose().getX(),
                    -poseInCar.getPose().getYaw()                                           //< rotation flipped
                                                                                  );
  // scaling with ratio
  pixel_coord.translation() *= getPixelRatio();

  return ManagedPose(oadrive::core::ExtendedPose2d(pixel_coord), IN_PIXEL);
}
/*
ManagedPose ManagedPose::transformCarToWorld(ManagedPose poseInCar, ManagedPose vehicle_pose)
{
  assert(poseInCar.getContext() == IN_CAR && "poseInCar not in Context IN_CAR!");
  assert(vehicle_pose.getContext() == IN_WORLD && "vehicle_pose not in Context IN_WORLD!");

  // use odometry information of vehicle_pose to obtain world coordinates
  oadrive::core::ExtendedPose2d pose_in_world;
  pose_in_world.pose() = vehicle_pose.getPose().pose()*(poseInCar.getPose().pose());
  ManagedPose managed_pose_in_world(pose_in_world, IN_WORLD);
  return managed_pose_in_world;
}

ManagedPose ManagedPose::transformPixelToWorld(ManagedPose poseInPixel, ManagedPose vehicle_pose)
{
  assert(poseInPixel.getContext() == IN_PIXEL && "poseInPixel not in Context IN_PIXEL!");
  assert(vehicle_pose.getContext() == IN_WORLD && "vehicle_pose not in Context IN_WORLD!");


  ManagedPose poseInCar = ManagedPose::transformPixelToCar(poseInPixel);
  ManagedPose poseInWorld = ManagedPose::transformCarToWorld(poseInCar, vehicle_pose);

  return poseInWorld;
}*/

string ManagedPose::toString()
{
  return "[" + oadrive::core::to_string((getPose().getX()))
      + ", " + oadrive::core::to_string(getPose().getY())
      + ", " + oadrive::core::to_string(getPose().getYaw())
      + ", " + ManagedPose::ContextNames.at(getContext()) + "]";
}

bool ManagedPose::readTransformationParameters(const string& filename)
{
  if (static_initialized)
    return true;

  TiXmlDocument doc(filename);
  if (!doc.LoadFile())
  {
    return false;
  }


  TiXmlElement* top_node = doc.FirstChildElement(XML_NODE_TOP);
  if (top_node == nullptr)
    return false;

  // read pixel ratio
  TiXmlElement* ratio_element = top_node->FirstChildElement(XML_ELEMENT_RATIO);
  if (ratio_element == nullptr)
    return false;

  if (ratio_element->QueryDoubleAttribute(XML_ELEMENT_RATIO_VALUE, &static_pixel_ratio) != TIXML_SUCCESS)
    return false;

  assert(static_pixel_ratio != 0 && "Error reading pixel ration from xml. export LANG=C");

  // read pixel ratio
  TiXmlElement* coordinate_element = top_node->FirstChildElement(XML_ELEMENT_ORIGIN);
  if (coordinate_element == nullptr)
    return false;

  if (coordinate_element->QueryDoubleAttribute(XML_ELEMENT_ORIGIN_XVALUE, &static_pixel_coordinate_origin.x) != TIXML_SUCCESS)
    return false;

  if (coordinate_element->QueryDoubleAttribute(XML_ELEMENT_ORIGIN_YVALUE, &static_pixel_coordinate_origin.y) != TIXML_SUCCESS)
    return false;

  static_initialized = true;

  // transfrom field of view rectangle to pixel coordinates
  oadrive::core::Position2d pos = transformCarToPixel(FIELD_OF_VISION_RECTANGLE_POS).getPose().getPose().translation();
  oadrive::core::Position2d size = FIELD_OF_VISION_RECTANGLE_SIZE.getPose().getPose().translation() * getPixelRatio();
  m_field_of_vision_rectangle_pixel = cv::Rect(pos.x(), pos.y(), size.x(), size.y());

  return true;
}

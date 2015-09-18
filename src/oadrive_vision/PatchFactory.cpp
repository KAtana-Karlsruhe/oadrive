#include "PatchFactory.h"

// std::getenv
#include <cstdlib>

#include <iostream>
#include <string>

// struct stat, S_ISDIR
#include <sys/stat.h>

#include <cmath>
#include <boost/lexical_cast.hpp>
#include <oadrive_core/Utility.h>

PatchFactory::Ptr PatchFactory::m_factory = PatchFactory::Ptr();

Patch::ConstPtr PatchFactory::getPatchByType(Patch::PatchType type, float rotation) const
{
  assert(isInitialize && "You forgot to initialize the PatchFactory");
  assert(type < Patch::PATCH_COUNT);

  // to read file from harddisk // assume rotation is normalized +/-pi/2
  _patch_rotation_type roundedRotationInDeg = std::round(180.0f*rotation/M_PI);

  //assert(abs(roundedRotationInDeg) <= 90 && "abs(roundedRotationInDeg) <= 90");

  return (m_patches[type]).at(roundedRotationInDeg);
}

TrackSegment PatchFactory::getTrackByType(Patch::PatchType type, float rotation) const
{
  assert(type < Patch::PATCH_COUNT);
  cv::Size trackSize;

  //Get base patch to receive the size of the patch
  Patch::ConstPtr basePatch = PatchFactory::getFactory()->getPatchByType(type, 0);
  trackSize = basePatch->getSize();

  //trackLocation is initial not used, users must translate is by themselve.
  ManagedPose trackLocation(0,0,0,ManagedPose::Context::IN_PIXEL);

  //get start and endJunction to build managedPose.
  cv::Point startJunction = PatchFactory::getFactory()->getBaseStartJunctionByType(type);
  cv::Point endJunction = PatchFactory::getFactory()->getBaseEndJunctionByType(type);
  ManagedPose managedStartJunction(startJunction.x, startJunction.y, rotation, ManagedPose::Context::IN_PIXEL);
  ManagedPose managedEndJunction(endJunction.x, endJunction.y, basePatch->getDeltaOrientation(), ManagedPose::Context::IN_PIXEL);
  managedEndJunction = managedEndJunction.rotate(managedStartJunction);

  //build basetrack
  TrackSegment baseTrack(type, managedStartJunction, managedEndJunction, trackLocation, trackSize, 0);
  return baseTrack;
}

bool PatchFactory::initialize()
{
  // get path to pattern folder and check that it exists and is a directory
  std::string patternFolder(std::getenv("PATTERNS"));
  struct stat sb;

  if (stat(patternFolder.c_str(), &sb) == -1)
  {
    std::cout << "Path '" << patternFolder << "' does not exist!";
    return false;
  }
  else if (!S_ISDIR(sb.st_mode))
  {
    std::cout << "Path '" << patternFolder << "' is no directory!";
    return false;
  }

  for(u_int8_t i = 0; i < (u_int8_t)Patch::PATCH_COUNT; ++i){
//    Patch::PatchType type = Patch::PATCH_NAMES.at( i);
    // set parameters dependent on patch type
    std::string patchType = Patch::PATCH_NAMES.at((Patch::PatchType)i);

    Patch::PatchType type = (Patch::PatchType) i;
    cv::Point2f start, end;

    PatchRotationContainer& current_patches = m_patches[i];



    float deltaOrientation = 0;

    if (type == Patch::SMALL_R_CURVE)
    {
      deltaOrientation = 18*M_PI/180.0f;     
    }
    else if(type == Patch::SMALL_L_CURVE)
    {
      deltaOrientation = -18*M_PI/180.0f;
    }
    //! TODO add other patches type

    start = getBaseStartJunctionByType(type);
    end = getBaseEndJunctionByType(type);

    for(_patch_rotation_type j = -90; j < 91; ++j)
    {

      // to read file from harddisk
      const _patch_rotation_type roundedRotationInDeg = j; //int(180.0f*j/M_PI);

      // set parameters dependent on rotation type
      std::string rotationType = "_rotated_" + oadrive::core::to_string(roundedRotationInDeg);
      // if rotation is 0 suffix is _rotated_0!

      // build path to png file and check that it exists and is a regular file
      std::string pathToPatch = patternFolder + patchType + rotationType + ".png";

      if((stat(pathToPatch.c_str(), &sb) != -1) && (S_ISREG(sb.st_mode)))
      {
        cv::Mat pattern = cv::imread(pathToPatch, 0);
        float rotationInRad = M_PI * roundedRotationInDeg / 180.0f;
        Patch::Ptr patch = std::make_shared<Patch>(pattern, type, start, end, rotationInRad, deltaOrientation);
        current_patches[roundedRotationInDeg] = patch;
      }
      else
      {
        std::cout <<"PatchFactory: could not load file" <<std::endl;
        return false;
      }
    }
    // TODO: quickfix to obtain tracksegments for extreme rotations
    for(int j = -180; j < -90; ++j)
    {
      _patch_rotation_type roundedRotationInDeg = j; //int(180.0f*j/M_PI);
      cv::Mat pattern = current_patches.at(0)->getPattern();
      float rotationInRad = M_PI * roundedRotationInDeg / 180.0f;
      Patch::Ptr patch = std::make_shared<Patch>(pattern, type, start, end, rotationInRad, deltaOrientation);
      current_patches[roundedRotationInDeg] = patch;
    }

    for(int j = 91; j < 181; ++j)
    {
      _patch_rotation_type roundedRotationInDeg = j; //int(180.0f*j/M_PI);
      cv::Mat pattern = current_patches.at(0)->getPattern();
      float rotationInRad = M_PI * roundedRotationInDeg / 180.0f;
      Patch::Ptr patch = std::make_shared<Patch>(pattern, type, start, end, rotationInRad, deltaOrientation);
      current_patches[roundedRotationInDeg] = patch;
    }
  }
  // set flag
  isInitialize = true;
  // success
  return true;
}



cv::Point2f PatchFactory::getBaseEndJunctionByType(Patch::PatchType type) const
{
  cv::Point2f end_junction;
  switch(type) {
    case Patch::STRAIGHT:
    {
      // measured in base picture
      end_junction = cv::Point(184,56);
      break;
    }
    case Patch::SMALL_R_CURVE:
    {
      // measured in base picture
      end_junction = cv::Point(206,68);
      break;
    }
    case Patch::SMALL_L_CURVE :
    {
      // measured in base picture
      end_junction = cv::Point (162,68);
      break;
    }
    case Patch::JUNCTION:
    case Patch::T_CROSS_LEFT:
    case Patch::T_CROSS_RIGHT:
    {
      // all junctions have same end pose
      end_junction = cv::Point(450,550);
      break;
    }
    default:
      std::cout << "Calculation of end junction not implemented yet for this Patch type!" << std::endl;
      throw 1;
  }
  if(ManagedPose::getPixelRatio() != Patch::INITIAL_PICTURE_PIXEL_RATIO){
    end_junction.x *= ManagedPose::getPixelRatio() / Patch::INITIAL_PICTURE_PIXEL_RATIO;
    end_junction.y *= ManagedPose::getPixelRatio() / Patch::INITIAL_PICTURE_PIXEL_RATIO;
  }

  return end_junction;
}

cv::Point2f PatchFactory::getBaseStartJunctionByType(Patch::PatchType type) const
{
  cv::Point2f start_junction;
  switch(type)
  {
    case Patch::STRAIGHT :
    {
      // measured in base picture
      start_junction = cv::Point2f(184,206);;
      break;
    }
    case Patch::SMALL_R_CURVE :
    {
      // measured in base picture
      start_junction = cv::Point2f(184,206);;
      break;
    }
    case Patch::SMALL_L_CURVE :
    {
      // measured in base picture
      start_junction = cv::Point2f(184,206);;
      break;
    }
    case Patch::JUNCTION:
    case Patch::T_CROSS_LEFT:
    case Patch::T_CROSS_RIGHT:
    {
      // all junctions have same start pose
      start_junction = cv::Point2f(300,450);
      break;
    }
    default:
      assert(false && "Calculation of start junction not implemented yet for this Patch type!");
      return cv::Point2f();
  }
  if(ManagedPose::getPixelRatio() != Patch::INITIAL_PICTURE_PIXEL_RATIO)
  {
    start_junction.x *= ManagedPose::getPixelRatio() / Patch::INITIAL_PICTURE_PIXEL_RATIO;
    start_junction.y *= ManagedPose::getPixelRatio() / Patch::INITIAL_PICTURE_PIXEL_RATIO;
  }

  return start_junction;
}

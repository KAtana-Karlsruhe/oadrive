#ifndef OADRIVE_VISION_PATCHFACTORY_H_INCLUDED
#define OADRIVE_VISION_PATCHFACTORY_H_INCLUDED

#include <opencv2/core/core.hpp>
#include <map>
#include <array>
#include "Patch.h"
#include "TrackSegment.h"

class PatchFactory
{
public:


  typedef int32_t _patch_rotation_type;
  typedef std::map<_patch_rotation_type, Patch::ConstPtr> PatchRotationContainer;
  typedef std::array<PatchRotationContainer, Patch::PATCH_COUNT> PatchesContainer;

  typedef std::shared_ptr<PatchFactory> Ptr;

  static Ptr getFactory()
  {
    if (m_factory)
      return m_factory;
    m_factory.reset(new PatchFactory());
    return m_factory;
  }

  virtual ~PatchFactory()
  {

  }

  Patch::ConstPtr getPatchByType(Patch::PatchType type, float rotation = 0.0f) const;

  TrackSegment getTrackByType(Patch::PatchType type, float rotation = 0.0f) const;

  cv::Point2f getBaseStartJunctionByType(Patch::PatchType type) const;
  cv::Point2f getBaseEndJunctionByType(Patch::PatchType type) const;


  Patch::ConstPtr getStraightBasePatch() const        { return getPatchByType(Patch::STRAIGHT); }
  Patch::ConstPtr getSmallRightCurveBasePatch()       { return getPatchByType(Patch::SMALL_R_CURVE); }
  Patch::ConstPtr getSmallLeftCurveBasePatch()        { return getPatchByType(Patch::SMALL_L_CURVE); }

  //! You need to call this function, returns false on file errors, otherwise true
  bool initialize();

private:

  PatchFactory()
    : isInitialize(false)
  {

  }

  static Ptr m_factory;

  bool isInitialize;

  PatchesContainer m_patches;
};

#endif

# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

ICMAKER_SET("oadrive_vision")

ICMAKER_ADD_SOURCES(
    IPM.cpp
    Patch.cpp
    PatchFactory.cpp
    PatchSelector.cpp
    Searchspace.cpp
    ManagedPose.cpp
    Matcher.cpp
    TrackSegment.cpp
    DrawUtil.cpp
    Stitcher.cpp
    ImagePreprocessor.cpp
    LocalMapWriter.cpp
)

ICMAKER_ADD_HEADERS(
    IPM.h
    Patch.h
    PatchFactory.h
    PatchSelector.h
    Searchspace.h
    ManagedPose.h
    Matcher.h
    TrackSegment.h
    DrawUtil.h
    Stitcher.h
    ImagePreprocessor.h
    LocalMapWriter.h
)

ICMAKER_LOCAL_CPPDEFINES(-DOADRIVE_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_OADRIVE_)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(EXPORT
  icl_core
  oadrive_core
  OpenCV
  tinyxml
)

ICMAKER_BUILD_LIBRARY()
ICMAKER_INSTALL_HEADERS(oadrive_vision)

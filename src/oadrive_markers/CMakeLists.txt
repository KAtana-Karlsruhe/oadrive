# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

ICMAKER_SET("oadrive_markers")

ICMAKER_ADD_SOURCES(
RoadSignDetector.cpp
)

ICMAKER_ADD_HEADERS(
RoadSignDetector.h
)

ICMAKER_LOCAL_CPPDEFINES(-DOADRIVE_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_OADRIVE_)

ICMAKER_INCLUDE_DIRECTORIES(${oadrive_INCLUDE_DIRS} ${aruco_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(EXPORT
  icl_core
  oadrive_core
  oadrive_vision
  OpenCV
  tinyxml
  aruco
)

ICMAKER_BUILD_LIBRARY()
ICMAKER_INSTALL_HEADERS(oadrive_markers)

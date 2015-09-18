// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-03-02
 *
 */
//----------------------------------------------------------------------
#ifndef OADRIVE_VISION_LOCAL_MAP_WRITER_H_INCLUDED
#define OADRIVE_VISION_LOCAL_MAP_WRITER_H_INCLUDED

#include <oadrive_core/Pose.h>
#include <oadrive_core/Trajectory2d.h>
#include <opencv/cv.h>

namespace oadrive {
namespace vision {


class LocalMapWriter
{
public:
  //! Image data
  typedef struct {std::string file; oadrive::core::Pose2d pose; double width; double height;} ImageData;

  //! Constructor
  LocalMapWriter()      {}
  LocalMapWriter(const LocalMapWriter& rhs) = delete;
  LocalMapWriter& operator=(const LocalMapWriter& rhs) = delete;

  //! Destruktor
  ~LocalMapWriter()     {}


  void addImage(const oadrive::core::Pose2d& pose, const std::string& image_path, double width = 200, double height = 200);
  void addTrajectory(const oadrive::core::Trajectory2d& trajectory);
  void addObstacle(const oadrive::core::Pose2d &pose);

  void write(const std::string& filename, double width = 800, double height = 600);

  void clear();

private:

  //! Images
  std::vector<ImageData> m_images;

  //! Trajectories
  std::vector<oadrive::core::Trajectory2d> m_trajectories;

  //! Obstacles
  std::vector<oadrive::core::Pose2d> m_obstacles;
};

} // end of ns
} // end of ns

#endif

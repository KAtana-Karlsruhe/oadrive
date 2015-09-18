// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-02-06
 *
 */
//----------------------------------------------------------------------

#include "LocalMapWriter.h"

#include <fstream>

namespace oadrive {
namespace vision {

double radToDeg(double rad)
{
  return rad*180.0/M_PI;
}

void LocalMapWriter::addImage(const core::Pose2d& pose, const std::string& image_path, double width, double height)
{
  m_images.push_back({image_path, pose, width, height});
}

void LocalMapWriter::addTrajectory(const core::Trajectory2d &trajectory)
{
  m_trajectories.push_back(trajectory);
}

void LocalMapWriter::addObstacle(const core::Pose2d &pose)
{
  m_obstacles.push_back(pose);
}

void LocalMapWriter::clear()
{
  m_images.clear();
  m_trajectories.clear();
  m_obstacles.clear();
}

void LocalMapWriter::write(const std::string& filename, double width, double height)
{
  std::ofstream ofs(filename.c_str());

  // calculate min/max values
  double x_min = std::numeric_limits< double >::max();
  double y_max = std::numeric_limits< double >::min();
  //double x_max = std::numeric_limits< double >::min();
  //double y_max = std::numeric_limits< double >::min();

  for (size_t i = 0; i < m_images.size(); ++i) {
    if(m_images[i].pose.translation().x() < x_min) x_min = m_images[i].pose.translation().x();
    if(m_images[i].pose.translation().y() > y_max) y_max = m_images[i].pose.translation().y();
    //if(m_images[i].pose.translation().x() > x_max) x_max = m_images[i].pose.translation().x();
    //if(m_images[i].pose.translation().y() > y_max) y_max = m_images[i].pose.translation().y();
  }

  // add offset
  double x_offset = 0;
  double y_offset = 0;
  if(x_min < 0) {
     x_offset = (-x_min + 5) * 100;
  }
  if(y_max > 0) {
    y_offset = -(y_max + 5) * 100;
  }

  std::stringstream header;
  header << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" <<std::endl
         <<"<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">" <<std::endl
         <<std::endl
         << "<svg xmlns=\"http://www.w3.org/2000/svg\"" <<std::endl
         <<"xmlns:xlink=\"http://www.w3.org/1999/xlink\" xmlns:ev=\"http://www.w3.org/2001/xml-events\"" <<std::endl
         <<"version=\"1.1\" baseProfile=\"full\"" <<std::endl
         <<"width=\"" <<width <<"mm\" height=\"" <<height <<"mm\">" <<std::endl;

  ofs <<header.str();

  // IMAGES
  for (size_t i = 0; i < m_images.size(); i++)
  {
    std::stringstream image_tag;
    image_tag <<"<image x=\"" <<m_images[i].pose.translation().x()*100.0 + x_offset <<"\" y=\"" <<-(m_images[i].pose.translation().y()*100.0 + y_offset) <<"\" width=\""
              <<m_images[i].width*100.0 <<"\" height=\"" <<m_images[i].height*100.0 <<"\"" <<std::endl
              <<"xlink:href=\"" <<m_images[i].file <<"\"" <<std::endl
              <<"transform=\"rotate(" <<-radToDeg(oadrive::core::PoseTraits<oadrive::core::Pose2d>::yaw(m_images[i].pose)) <<","
                 <<m_images[i].pose.translation().x()*100.0 + x_offset <<"," <<-(m_images[i].pose.translation().y()*100.0 + y_offset) <<")\">" <<std::endl
              <<"</image>" <<std::endl;

    ofs <<image_tag.str();
  }

  // TRAJECTORIES
  for (size_t i = 0; i < m_trajectories.size(); i++)
  {
    ofs <<std::endl;

    std::stringstream trajectory_tags;

    u_int32_t rgb[3] = { 64* (i+1 % 4) + 63, 64* (i+3 % 4) + 63, 64* (i % 4) + 63};

    for (size_t p = 0; p < m_trajectories[i].size(); p++)
    {
      trajectory_tags <<"<circle id=\"" <<i*10000 + p
         <<"\" cx=\"" <<m_trajectories[i][p].getPose().translation().x()*100.0 + x_offset
         <<"\" cy=\"" <<-(m_trajectories[i][p].getPose().translation().y()*100.0 + y_offset)
         <<"\" r=\"1\" fill=\"rgb(" <<rgb[0] <<", " <<rgb[1] <<", " <<rgb[2] <<")\" stroke=\"none\" />" <<std::endl;
    }

    ofs <<trajectory_tags.str() <<std::endl;
  }

  // OBSTACLES
  for (size_t i = 0; i < m_obstacles.size(); i++)
  {
    std::stringstream obstacle_tag;
    // 267 308
    obstacle_tag <<"<image x=\"" <<m_obstacles[i].translation().x()*100.0 + x_offset <<"\" y=\"" <<-(m_obstacles[i].translation().y()*100.0 + y_offset) <<"\" width=\""
              <<0.2*100.0 <<"\" height=\"" <<0.3*100.0 <<"\"" <<std::endl
              <<"xlink:href=\"" << "/tmp/pylon.png" <<"\"" <<std::endl
              <<">" <<std::endl
              <<"</image>" <<std::endl;

    ofs <<obstacle_tag.str();
  }


  // FOOTER
  ofs <<"</svg>" <<std::endl;

}

} // end of ns
} // end of ns

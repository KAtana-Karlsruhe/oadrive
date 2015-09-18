#include "DrawUtil.h"
#include <oadrive_core/Utility.h>

using namespace std;
using namespace cv;

const string OUTPUT_PATH = "/tmp/lanetracker/";


// BGR colors
const Scalar DrawUtil::DARKRED(0, 0, 139); // searchspace
const Scalar DrawUtil::DARKBLUE(139, 0, 0); //straight
const Scalar DrawUtil::DARKGREEN(0, 100, 0); // r_curve
const Scalar DrawUtil::GOLD(0,215,255); // l_curve
const Scalar DrawUtil::MAGENTA(255, 0, 255); // stitch

const Scalar DrawUtil::YELLOW(0, 255, 255);
const Scalar DrawUtil::ORANGE(36, 127, 255);
const Scalar DrawUtil::RED(0, 0, 255);

const Scalar DrawUtil::GREEN(0, 255, 0);
const Scalar DrawUtil::SEAGREEN(87, 139, 46);
const Scalar DrawUtil::CYAN(139, 139, 0);

const Scalar DrawUtil::GRAY(130, 130, 130);

using namespace oadrive::core;

// warning: if colors are added, adjust draw methods and % operator
const int DrawUtil::COLOR_COUNT = 10;
Scalar colors[] = {DrawUtil::DARKBLUE, DrawUtil::DARKGREEN, DrawUtil::GOLD, DrawUtil::MAGENTA, DrawUtil::YELLOW,
                   DrawUtil::ORANGE, DrawUtil::GREEN, DrawUtil::SEAGREEN, DrawUtil::CYAN, DrawUtil::GRAY};

DrawUtil::DrawUtil()
{
}

void DrawUtil::drawPoint(Mat &input, Point position, int id)
{
  assert(input.cols > 0 && input.rows > 0 && "input.size().x > 0 && input.size().y > 0 in drawPoint");
  circle(input, position, 1, colors[id%DrawUtil::COLOR_COUNT],-1);
}

void DrawUtil::drawManagedPose(Mat &input, ManagedPose point) {
    assert(input.cols > 0 && input.rows > 0 && "input.size().x > 0 && input.size().y > 0 in drawManagedPose");
    Point position(point.getX(), point.getY());
    circle(input, position, 1, colors[8],-1);
}

void DrawUtil::drawTrack(Mat &input, TrackSegment track, int id) {
    assert(input.cols > 0 && input.rows > 0 && "input.size().x > 0 && input.size().y > 0 in drawTrack");

    int index = id%DrawUtil::COLOR_COUNT;
    Point start(track.getStartJunction().getPose().getX(), track.getStartJunction().getPose().getY());
    Point end(track.getEndJunction().getPose().getX(), track.getEndJunction().getPose().getY());

    // draw junctions
    circle(input, start, 1, colors[index],-1);
    putTextAt(input, start, "S " + std::to_string(track.getStartJunction().getThetaInDegree()), index);
    circle(input, end, 1, colors[index],-1);
    putTextAt(input, end, "E", index);

    // draw info
    Point info(start.x - track.getSize().width/2, start.y + 30);
    assert (track.getPatchType() < Patch::PATCH_COUNT);
    putTextAt(input, info, std::to_string(id) + ": " + Patch::PATCH_NAMES.at(track.getPatchType()) + " " + std::to_string(track.getMatchingValue()), index);
}


void DrawUtil::drawSearchspace(Mat &input, RotatedRect searchspace, int index)
{
  assert(input.cols > 0 && input.rows > 0 && "input.size().x > 0 && input.size().y > 0 in drawSearchspace");
  Point2f vertices[4];
  searchspace.points(vertices);
  for (int i = 0; i < 4; i++) {
    line(input, vertices[i], vertices[(i+1)%4], colors[index % DrawUtil::COLOR_COUNT]);
  }
  Point info(searchspace.center.x + 20, searchspace.center.y + 20);
  putTextAt(input, info, "rot: " + oadrive::core::to_string(searchspace.angle), index);
}

void DrawUtil::drawSearchTolerance(cv::Mat &input, const Matcher::SearchTolerance& tolerance, int index)
{
  assert(input.cols > 0 && input.rows > 0 && "input.size().x > 0 && input.size().y > 0 in drawSearchspace");
  Point2f vertices[4];

  Pose2d p[4];
  Pose2d transf[4];
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(p[0], tolerance.width/2, tolerance.height/2, 0.0);
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(p[1], -tolerance.width/2, tolerance.height/2, 0.0);
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(p[2], -tolerance.width/2, -tolerance.height/2, 0.0);
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(p[3], tolerance.width/2, -tolerance.height/2, 0.0);

  for (size_t i = 0; i < 4; i++)
  {
    transf[i] = tolerance.search_pose * p[i];
    vertices[i].x = transf[i].translation().x();
    vertices[i].y = transf[i].translation().y();
  }

  for (int i = 0; i < 4; i++) {
    line(input, vertices[i], vertices[(i+1)%4], colors[index % DrawUtil::COLOR_COUNT]);
  }

}

void DrawUtil::drawQuadrangle(cv::Mat& input, std::array<oadrive::core::Position2d, 4> corners)
{
  assert(input.cols > 0 && input.rows > 0 && "input.size().x > 0 && input.size().y > 0 in drawQuadrangle");
  Point2f vertices[4];


  for (size_t i = 0; i < 4; i++)
  {
    vertices[i].x = corners[i](0,0);
    vertices[i].y = corners[i](1,0);
  }

  for (int i = 0; i < 4; i++) {
    line(input, vertices[i], vertices[(i+1)%4], DARKBLUE);
    }
}

void DrawUtil::putTextAt(Mat &input, const Point& position, const string& text, int color)
{
  putText(input, text, position, FONT_HERSHEY_PLAIN, 1.5, colors[color % DrawUtil::COLOR_COUNT]);
}

void DrawUtil::writeImageToFile(const Mat& image, const string& suffix, bool addTimestamp)
{
  static int counter = 1000;
  string finalPath;
  if (addTimestamp)
  {
  // source at http://stackoverflow.com/a/997566/1176596
  std::time_t now = std::time(NULL);
  std::tm * ptm = std::localtime(&now);
  char buffer[32];
  // Format: 2015-01-22-23_59_00
  // no ":" sign in windows files...
  std::strftime(buffer, 32, "%F-%H_%M_%S", ptm);

  // write image as png
  string timestamp(buffer);
    finalPath = OUTPUT_PATH + timestamp + "_" + std::to_string(counter) + suffix + ".png";
    counter++;
  }
  else
  {
    finalPath = OUTPUT_PATH + suffix + ".png";
  }
  imwrite(finalPath, image);
}

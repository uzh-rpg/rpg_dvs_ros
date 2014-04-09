#include "ros/ros.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "dvs_calibration_dots");

  ros::NodeHandle nh;

  // load parameters from rosparam
  int resolution = 500;
  int dots = 4;
  double radius = 25;
  bool blinking = true;
  int wait_time_ms = 500;
  string window_name = "intrinsic calibration pattern";

  // initialize pattern points
  std::vector<Eigen::Vector3d> pattern_points;
  for (int i=0; i<dots; i++) {
    for (int j=0; j<dots; j++) {
      double x = (i - (dots-1)/2.0)/(0.5*(dots-1));
      double y = (j - (dots-1)/2.0)/(0.5*(dots-1));
      pattern_points.push_back(Eigen::Vector3d(x, y, 10.0));
      std::cout << Eigen::Vector3d(x, y, 2.0) << std::endl;
    }
  }

  // list of orientations
  std::vector<Eigen::Matrix3d> pattern_orientations;
  for (int i=-45; i<=45; i+=15) {
    for (int j=-45; j<=45; j+=15) {
      double roll = i/180.0 * M_PI;
      double pitch = j/180.0 * M_PI;
      Eigen::Matrix3d R;
      R = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
      pattern_orientations.push_back(R);
      std::cout << R << std::endl;
    }
  }

  Mat black = Mat(resolution, resolution, CV_8UC3);
  black = Scalar(0, 0, 0);



  namedWindow(window_name, WINDOW_NORMAL | CV_GUI_NORMAL);

  int orientation_id = 0;
  while (ros::ok()) {
    // transform all points
    std::vector<Eigen::Vector2d> image_points;
    for (int i=0; i<pattern_points.size(); i++) {
      Eigen::Vector3d p = pattern_orientations[orientation_id] * pattern_points[i];
      image_points.push_back(Eigen::Vector2d(p.x()/p.z(), p.y()/p.z()));
    }

    // find minimum and maximum to scale appropriately
    Eigen::Vector2d min = image_points[0];
    Eigen::Vector2d max = image_points[0];
    for (int i=1; i<image_points.size(); i++) {
      min.x() = std::min(min.x(), image_points[i].x());
      min.y() = std::min(min.y(), image_points[i].y());
      max.x() = std::max(max.x(), image_points[i].x());
      max.y() = std::max(max.y(), image_points[i].y());
    }

    std::cout << min << std::endl;
    std::cout << max << std::endl;
    std::cout << std::endl;


    double width = max.x() - min.x();
    double height = max.y() - min.y();

    std::cout << width << std::endl;

    double factor = (resolution - 2*radius)/std::max(width, height);
    std::cout << factor << std::endl;

    Mat image = Mat(resolution, resolution, CV_8UC3);
    image = Scalar(0, 0, 0);

    for (int i=0; i<image_points.size(); i++) {
      circle(image, Point(radius + (image_points[i].x()-min.x())*factor,
                          radius + (image_points[i].y()-min.y())*factor),
             radius, Scalar(255, 255, 255), CV_FILLED);
    }

    orientation_id++;
    if (orientation_id >= pattern_orientations.size())
      orientation_id = 0;

    imshow(window_name, image);
    waitKey(wait_time_ms);

    if (blinking) {
      imshow(window_name, black);
      waitKey(wait_time_ms);
    }
  }

  return 0;
}

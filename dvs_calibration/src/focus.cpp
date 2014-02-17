#include "ros/ros.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "dvs_calibration_focus");

  ros::NodeHandle nh();

  // load parameters from rosparam
  int resolution = 500;
  bool blinking = false;
  int wait_time_ms = 100;
  string window_name = "focus calibration pattern";

  Mat black = Mat(resolution, resolution, CV_8UC3);
  black = Scalar(0, 0, 0);

  Mat image = Mat(resolution, resolution, CV_8UC3);
  image = Scalar(0, 0, 0);

  double size = resolution / 2.0;
  bool white = false;
  while (size > 1) {
    Scalar color;
    if (white) {
      white = false;
      color = Scalar(255, 255, 255);
    } else {
      white = true;
      color = Scalar(0, 0, 0);
    }
    rectangle(image, Point(resolution/2.0 - size, resolution/2.0 - size), Point(resolution/2.0 + size, resolution/2.0 + size), color, CV_FILLED);

    size /= 1.2;
  }

  namedWindow(window_name, WINDOW_NORMAL | CV_GUI_NORMAL);

  if (blinking)
  {
    while (ros::ok()) {
      imshow(window_name, image);
      waitKey(wait_time_ms);
      imshow(window_name, black);
      waitKey(wait_time_ms);
    }
  }
  else {
    imshow(window_name, image);
    waitKey(0);
  }

  return 0;
}

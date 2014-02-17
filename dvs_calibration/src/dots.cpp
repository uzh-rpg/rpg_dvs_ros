#include "ros/ros.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "dvs_calibration_dots");

  ros::NodeHandle nh();

  // load parameters from rosparam
  int resolution = 500;
  int dots = 7;
  double radius = 15;
  bool blinking = false;
  int wait_time_ms = 100;
  string window_name = "intrinsic calibration pattern";

  Mat black = Mat(resolution, resolution, CV_8UC3);
  black = Scalar(0, 0, 0);

  Mat image = Mat(resolution, resolution, CV_8UC3);
  image = Scalar(0, 0, 0);

  double d = resolution / (double) dots;
  for (int i=0; i<dots; i++) {
      for (int j=0; j<dots; j++) {
          circle(image, Point(d/2 + i*d, d/2 + j*d), radius, Scalar(255, 255, 255), CV_FILLED);
      }
  }

  namedWindow(window_name, WINDOW_NORMAL | CV_GUI_NORMAL);

  while (ros::ok()) {
    imshow(window_name, image);
    waitKey(wait_time_ms);

    if (blinking) {
      imshow(window_name, black);
      waitKey(wait_time_ms);
    }
  }

  return 0;
}

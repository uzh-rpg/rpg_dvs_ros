#include "ros/ros.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include "dvs_calibration/DVSCalibration.h"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "dvs_calibration");

  ros::NodeHandle nh();

  // load parameters from rosparam
  int dots = 7;

  DVSCalibration calib(dots);

  // register callback


  ros::spin();

  return 0;
}

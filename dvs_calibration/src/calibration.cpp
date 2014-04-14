#include "ros/ros.h"

#include "dvs_calibration/dvs_calibration.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dvs_calibration");

  DvsCalibration dvs_calibration;

  ros::spin();

  return 0;
}

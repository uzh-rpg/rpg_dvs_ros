// This file is part of DVS-ROS - the RPG DVS ROS Package

#include "ros/ros.h"

#include "dvs_calibration/stereo_dvs_calibration.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stereo_dvs_calibration");

  dvs_calibration::StereoDvsCalibration stereo_dvs_calibration;

  ros::spin();

  return 0;
}

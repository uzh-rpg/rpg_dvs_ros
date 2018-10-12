// This file is part of DVS-ROS - the RPG DVS ROS Package

#include "ros/ros.h"

#include "dvs_calibration/mono_dvs_calibration.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mono_dvs_calibration");

  dvs_calibration::MonoDvsCalibration mono_dvs_calibration;

  ros::spin();

  return 0;
}

// This file is part of DVS-ROS - the RPG DVS ROS Package

#include <ros/ros.h>

#include "davis_ros_driver/driver.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "davis_ros_driver");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  davis_ros_driver::DavisRosDriver* driver = new davis_ros_driver::DavisRosDriver(nh, nh_private);

  ros::spin();
  driver->dataStop();
  return 0;
}

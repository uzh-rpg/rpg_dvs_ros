// This file is part of DVS-ROS - the RPG DVS ROS Package

#include <ros/ros.h>
#include "dvs_ros_driver/driver.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dvs_ros_driver");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  dvs_ros_driver::DvsRosDriver* driver = new dvs_ros_driver::DvsRosDriver(nh, nh_private);

  ros::spin();
  return 0;
}

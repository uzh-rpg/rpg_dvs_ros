// This file is part of DVS-ROS - the RPG DVS ROS Package

#include <pluginlib/class_list_macros.h>

#include "davis_ros_driver/driver_nodelet.h"

namespace davis_ros_driver
{

void DavisRosDriverNodelet::onInit()
{
  driver_ = new davis_ros_driver::DavisRosDriver(getNodeHandle(), getPrivateNodeHandle());

  NODELET_INFO_STREAM("Initialized " <<  getName() << " nodelet.");
}

#ifndef PLUGINLIB_EXPORT_CLASS
PLUGINLIB_DECLARE_CLASS(davis_ros_driver, DavisRosDriverNodelet, davis_ros_driver::DavisRosDriverNodelet, nodelet::Nodelet);
#else
PLUGINLIB_EXPORT_CLASS(davis_ros_driver::DavisRosDriverNodelet, nodelet::Nodelet);
#endif

}

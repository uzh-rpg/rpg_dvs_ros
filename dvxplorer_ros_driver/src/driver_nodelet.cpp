// This file is part of DVS-ROS - the RPG DVS ROS Package

#include "dvxplorer_ros_driver/driver_nodelet.h"

#include <pluginlib/class_list_macros.h>

namespace dvxplorer_ros_driver {

void DvxplorerRosDriverNodelet::onInit() {
	driver_ = new dvxplorer_ros_driver::DvxplorerRosDriver(getNodeHandle(), getPrivateNodeHandle());

	NODELET_INFO_STREAM("Initialized " << getName() << " nodelet.");
}

#ifndef PLUGINLIB_EXPORT_CLASS
PLUGINLIB_DECLARE_CLASS(
	dvxplorer_ros_driver, DvxplorerRosDriverNodelet, dvxplorer_ros_driver::DvxplorerRosDriverNodelet, nodelet::Nodelet);
#else
PLUGINLIB_EXPORT_CLASS(dvxplorer_ros_driver::DvxplorerRosDriverNodelet, nodelet::Nodelet);
#endif

} // namespace dvxplorer_ros_driver

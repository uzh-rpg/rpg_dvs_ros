// This file is part of DVS-ROS - the RPG DVS ROS Package

#pragma once

#include "dvxplorer_ros_driver/driver.h"

#include <nodelet/nodelet.h>

namespace dvxplorer_ros_driver {

class DvxplorerRosDriverNodelet : public nodelet::Nodelet {
public:
	virtual void onInit();

private:
	dvxplorer_ros_driver::DvxplorerRosDriver *driver_;
};

} // namespace dvxplorer_ros_driver

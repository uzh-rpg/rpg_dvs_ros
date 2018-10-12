// This file is part of DVS-ROS - the RPG DVS ROS Package

#pragma once

#include <nodelet/nodelet.h>

#include "dvs_ros_driver/driver.h"

namespace dvs_ros_driver {

class DvsRosDriverNodelet : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  dvs_ros_driver::DvsRosDriver* driver_;
};

}

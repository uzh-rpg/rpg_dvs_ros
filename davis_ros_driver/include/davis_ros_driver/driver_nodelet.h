// This file is part of DVS-ROS - the RPG DVS ROS Package

#pragma once

#include <nodelet/nodelet.h>

#include "davis_ros_driver/driver.h"

namespace davis_ros_driver {

class DavisRosDriverNodelet : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  davis_ros_driver::DavisRosDriver* driver_;
};

}

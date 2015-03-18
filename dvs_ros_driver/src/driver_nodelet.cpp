// This file is part of DVS-ROS - the RPG DVS ROS Package
//
// DVS-ROS is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// DVS-ROS is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with DVS-ROS.  If not, see <http://www.gnu.org/licenses/>.

#include <pluginlib/class_list_macros.h>

#include "dvs_ros_driver/driver_nodelet.h"

namespace dvs_ros_driver
{

void DvsRosDriverNodelet::onInit()
{
  driver_ = new dvs_ros_driver::DvsRosDriver(getNodeHandle(), getPrivateNodeHandle());

  NODELET_INFO_STREAM("Initialized " <<  getName() << " nodelet.");
}

PLUGINLIB_DECLARE_CLASS(dvs_ros_driver, DvsRosDriverNodelet, dvs_ros_driver::DvsRosDriverNodelet, nodelet::Nodelet);

}

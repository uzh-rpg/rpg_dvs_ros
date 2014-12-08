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

#ifndef DVS_ROS_DRIVER_H_
#define DVS_ROS_DRIVER_H_

#include <ros/ros.h>

// boost
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <std_msgs/Empty.h>

// DVS driver
#include <dvs_driver/dvs_driver.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dvs_ros_driver/DVS_ROS_DriverConfig.h>

// camera info manager
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

namespace dvs_ros_driver {

class DvsRosDriver {
public:
  DvsRosDriver();
  ~DvsRosDriver();

private:
  void change_dvs_parameters();
  void callback(dvs_ros_driver::DVS_ROS_DriverConfig &config, uint32_t level);
  void readout();

  ros::NodeHandle nh;
  ros::Publisher event_array_pub;
  ros::Publisher camera_info_pub;
  dvs::DVS_Driver *driver;

  dynamic_reconfigure::Server<dvs_ros_driver::DVS_ROS_DriverConfig> server;
  dynamic_reconfigure::Server<dvs_ros_driver::DVS_ROS_DriverConfig>::CallbackType f;

  ros::Subscriber reset_sub;
  void reset_timestamps(std_msgs::Empty msg);

  boost::thread* parameter_thread;
  boost::thread* readout_thread;

  boost::posix_time::time_duration delta;

  dvs_ros_driver::DVS_ROS_DriverConfig current_config;
  camera_info_manager::CameraInfoManager* cameraInfoManager;

  bool parameter_update_required;

};

} // namespace

#endif // DVS_ROS_DRIVER_H_

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

#pragma once

#include <ros/ros.h>
#include <string>

// boost
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <std_msgs/Empty.h>

// DVS driver
#include <libcaer/libcaer.h>
#include <libcaer/devices/dvs128.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dvs_ros_driver/DVS_ROS_DriverConfig.h>

// camera info manager
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

namespace dvs_ros_driver {

class DvsRosDriver {
public:
  DvsRosDriver(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  ~DvsRosDriver();

private:
  void changeDvsParameters();
  void callback(dvs_ros_driver::DVS_ROS_DriverConfig &config, uint32_t level);
  void readout();
  void resetTimestamps();

  ros::NodeHandle nh_;
  ros::Publisher event_array_pub_;
  ros::Publisher camera_info_pub_;
  caerDeviceHandle dvs128_handle;

  volatile bool running_;

  boost::shared_ptr<dynamic_reconfigure::Server<dvs_ros_driver::DVS_ROS_DriverConfig> > server_;
  dynamic_reconfigure::Server<dvs_ros_driver::DVS_ROS_DriverConfig>::CallbackType dynamic_reconfigure_callback_;

  ros::Subscriber reset_sub_;
  void resetTimestampsCallback(std_msgs::Empty msg);

  boost::shared_ptr<boost::thread> parameter_thread_;
  boost::shared_ptr<boost::thread> readout_thread_;

  boost::posix_time::time_duration delta_;

  dvs_ros_driver::DVS_ROS_DriverConfig current_config_;
  camera_info_manager::CameraInfoManager* camera_info_manager_;

  struct caer_dvs128_info dvs128_info_;
  std::string device_id_;

  ros::Time reset_time_;

  ros::Timer timestamp_reset_timer_;
  void resetTimerCallback(const ros::TimerEvent& te);

  bool parameter_update_required_;

};

} // namespace

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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

// DAVIS driver
#include <libcaer/libcaer.h>
#include <libcaer/devices/davis.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <davis_ros_driver/DAVIS_ROS_DriverConfig.h>

// camera info manager
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

namespace davis_ros_driver {

class DavisRosDriver {
public:
  DavisRosDriver(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  ~DavisRosDriver();

  static void onDisconnectUSB(void*);   
  
private:
  void changeDvsParameters();
  void callback(davis_ros_driver::DAVIS_ROS_DriverConfig &config, uint32_t level);
  void readout();
  void resetTimestamps();
  void caerConnect();

  ros::NodeHandle nh_;
  ros::Publisher event_array_pub_;
  ros::Publisher camera_info_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher image_pub_;
  caerDeviceHandle davis_handle_;
  
  std::string ns;

  volatile bool running_;

  boost::shared_ptr<dynamic_reconfigure::Server<davis_ros_driver::DAVIS_ROS_DriverConfig> > server_;
  dynamic_reconfigure::Server<davis_ros_driver::DAVIS_ROS_DriverConfig>::CallbackType dynamic_reconfigure_callback_;

  ros::Subscriber reset_sub_;
  void resetTimestampsCallback(const std_msgs::Empty::ConstPtr& msg);

  ros::Subscriber imu_calibration_sub_;
  void imuCalibrationCallback(const std_msgs::Empty::ConstPtr& msg);
  int imu_calibration_sample_size_;
  bool imu_calibration_running_;
  std::vector<sensor_msgs::Imu> imu_calibration_samples_;
  sensor_msgs::Imu bias;
  void updateImuBias();
  template <typename T> int sgn(T val) {
      return (T(0) < val) - (val < T(0));
  }

  ros::Subscriber snapshot_sub_;
  void snapshotCallback(const std_msgs::Empty::ConstPtr& msg);

  boost::shared_ptr<boost::thread> parameter_thread_;
  boost::shared_ptr<boost::thread> readout_thread_;

  boost::posix_time::time_duration delta_;

  davis_ros_driver::DAVIS_ROS_DriverConfig current_config_;
  camera_info_manager::CameraInfoManager* camera_info_manager_;

  struct caer_davis_info davis_info_;
  std::string device_id_;
  uint16_t device_type_;

  ros::Time reset_time_;

  static constexpr double STANDARD_GRAVITY = 9.81;

  ros::Timer timestamp_reset_timer_;
  void resetTimerCallback(const ros::TimerEvent& te);

  bool parameter_update_required_;
  bool parameter_bias_update_required_;

};

} // namespace

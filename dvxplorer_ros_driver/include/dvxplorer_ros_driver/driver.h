// This file is part of DVS-ROS - the RPG DVS ROS Package

#pragma once

#include <ros/ros.h>
#include <string>

// boost
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Time.h>

// DVXplorer driver
#include <libcaer/devices/dvxplorer.h>
#include <libcaer/libcaer.h>

// dynamic reconfigure
#include <dvxplorer_ros_driver/DVXplorer_ROS_DriverConfig.h>
#include <dynamic_reconfigure/server.h>

// camera info manager
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>

namespace dvxplorer_ros_driver {

class DvxplorerRosDriver {
public:
	DvxplorerRosDriver(ros::NodeHandle &nh, ros::NodeHandle nh_private);
	~DvxplorerRosDriver();

	void dataStop();

	static void onDisconnectUSB(void *);

private:
	void callback(dvxplorer_ros_driver::DVXplorer_ROS_DriverConfig &config, uint32_t level);
	void readout();
	void resetTimestamps();
	void caerConnect();

	ros::NodeHandle nh_;
	ros::Publisher event_array_pub_;
	ros::Publisher camera_info_pub_;
	ros::Publisher imu_pub_;
	caerDeviceHandle dvxplorer_handle_;

	std::string ns;

	std::atomic_bool running_;

	boost::shared_ptr<dynamic_reconfigure::Server<dvxplorer_ros_driver::DVXplorer_ROS_DriverConfig>> server_;
	dynamic_reconfigure::Server<dvxplorer_ros_driver::DVXplorer_ROS_DriverConfig>::CallbackType
		dynamic_reconfigure_callback_;

	ros::Subscriber reset_sub_;
	ros::Publisher reset_pub_;
	void resetTimestampsCallback(const std_msgs::Time::ConstPtr &msg);

	ros::Subscriber imu_calibration_sub_;
	void imuCalibrationCallback(const std_msgs::Empty::ConstPtr &msg);
	std::atomic_bool imu_calibration_running_;
	int imu_calibration_sample_size_;
	std::vector<sensor_msgs::Imu> imu_calibration_samples_;
	sensor_msgs::Imu bias;
	void updateImuBias();

	template<typename T>
	int sgn(T val) {
		return (T(0) < val) - (val < T(0));
	}

	boost::shared_ptr<boost::thread> readout_thread_;

	boost::posix_time::time_duration delta_;

	std::atomic_int streaming_rate_;
	std::atomic_int max_events_;

	std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

	struct caer_dvx_info dvxplorer_info_;
	bool master_;
	std::string device_id_;

	ros::Time reset_time_;

	static constexpr double STANDARD_GRAVITY = 9.81;

	ros::Timer timestamp_reset_timer_;
	void resetTimerCallback(const ros::TimerEvent &te);
};

} // namespace dvxplorer_ros_driver

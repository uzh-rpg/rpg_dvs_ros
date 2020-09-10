// This file is part of DVS-ROS - the RPG DVS ROS Package

#include "dvxplorer_ros_driver/driver.h"

#include <std_msgs/Int32.h>

namespace dvxplorer_ros_driver {

DvxplorerRosDriver::DvxplorerRosDriver(ros::NodeHandle &nh, ros::NodeHandle nh_private) :
	nh_(nh), imu_calibration_running_(false) {
	// load parameters
	nh_private.param<std::string>("serial_number", device_id_, "");
	nh_private.param<bool>("master", master_, true);
	double reset_timestamps_delay;
	nh_private.param<double>("reset_timestamps_delay", reset_timestamps_delay, -1.0);
	nh_private.param<int>("imu_calibration_sample_size", imu_calibration_sample_size_, 1000);

	// initialize IMU bias
	nh_private.param<double>("imu_bias/ax", bias.linear_acceleration.x, 0.0);
	nh_private.param<double>("imu_bias/ay", bias.linear_acceleration.y, 0.0);
	nh_private.param<double>("imu_bias/az", bias.linear_acceleration.z, 0.0);
	nh_private.param<double>("imu_bias/wx", bias.angular_velocity.x, 0.0);
	nh_private.param<double>("imu_bias/wy", bias.angular_velocity.y, 0.0);
	nh_private.param<double>("imu_bias/wz", bias.angular_velocity.z, 0.0);

	// set namespace
	ns = ros::this_node::getNamespace();
	if (ns == "/") {
		ns = "/dvs";
	}

	event_array_pub_ = nh_.advertise<dvs_msgs::EventArray>(ns + "/events", 10);
	camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(ns + "/camera_info", 1);
	imu_pub_         = nh_.advertise<sensor_msgs::Imu>(ns + "/imu", 10);

	// reset timestamps is publisher as master, subscriber as slave
	if (master_) {
		reset_pub_ = nh_.advertise<std_msgs::Time>((ns + "/reset_timestamps").c_str(), 1);
	}
	else {
		reset_sub_
			= nh_.subscribe((ns + "/reset_timestamps").c_str(), 1, &DvxplorerRosDriver::resetTimestampsCallback, this);
	}

	// Open device.
	caerConnect();

	// Dynamic reconfigure
	// Will call callback, which will pass stored config to device.
	dynamic_reconfigure_callback_ = boost::bind(&DvxplorerRosDriver::callback, this, _1, _2);
	server_.reset(new dynamic_reconfigure::Server<dvxplorer_ros_driver::DVXplorer_ROS_DriverConfig>(nh_private));
	server_->setCallback(dynamic_reconfigure_callback_);

	imu_calibration_sub_
		= nh_.subscribe((ns + "/calibrate_imu").c_str(), 1, &DvxplorerRosDriver::imuCalibrationCallback, this);

	// start timer to reset timestamps for synchronization
	if (reset_timestamps_delay > 0.0) {
		timestamp_reset_timer_
			= nh_.createTimer(ros::Duration(reset_timestamps_delay), &DvxplorerRosDriver::resetTimerCallback, this);
		ROS_INFO("Started timer to reset timestamps on master DVS for synchronization (delay=%3.2fs).",
			reset_timestamps_delay);
	}
}

DvxplorerRosDriver::~DvxplorerRosDriver() {
	if (running_) {
		ROS_INFO("shutting down threads");
		running_ = false;
		readout_thread_->join();
		ROS_INFO("threads stopped");
		caerLog(CAER_LOG_ERROR, "destructor", "data stop now");
		caerDeviceDataStop(dvxplorer_handle_);
		caerDeviceClose(&dvxplorer_handle_);
	}
}

void DvxplorerRosDriver::dataStop() {
	caerLog(CAER_LOG_INFO, "Exiting from driver node", "executing data stop");
	ROS_INFO("Exiting from driver node, executing data stop");
	caerDeviceDataStop(dvxplorer_handle_);
	caerDeviceClose(&dvxplorer_handle_);
}

void DvxplorerRosDriver::caerConnect() {
	// start driver
	bool device_is_running = false;
	while (!device_is_running) {
		const char *serial_number_restrict = (device_id_.empty()) ? nullptr : device_id_.c_str();

		if (serial_number_restrict) {
			ROS_WARN("Requested serial number: %s", device_id_.c_str());
		}

		dvxplorer_handle_ = caerDeviceOpen(1, CAER_DEVICE_DVXPLORER, 0, 0, serial_number_restrict);

		// was opening successful?
		device_is_running = (dvxplorer_handle_ != nullptr);

		if (!device_is_running) {
			ROS_WARN("Could not find DVXplorer. Will retry every second.");
			ros::Duration(1.0).sleep();
			ros::spinOnce();
		}

		if (!ros::ok()) {
			return;
		}
	}

	dvxplorer_info_ = caerDVXplorerInfoGet(dvxplorer_handle_);
	device_id_      = "DVXplorer-" + std::string(dvxplorer_info_.deviceSerialNumber);

	ROS_INFO("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Firmware: %d, Logic: %d.\n",
		dvxplorer_info_.deviceString, dvxplorer_info_.deviceID, dvxplorer_info_.deviceIsMaster,
		dvxplorer_info_.dvsSizeX, dvxplorer_info_.dvsSizeY, dvxplorer_info_.firmwareVersion,
		dvxplorer_info_.logicVersion);

	if (master_ && !dvxplorer_info_.deviceIsMaster) {
		ROS_WARN("Device %s should be master, but is not!", device_id_.c_str());
	}

	// Send the default configuration before using the device.
	// No configuration is sent automatically!
	caerDeviceSendDefaultConfig(dvxplorer_handle_);

	// spawn threads
	running_ = true;
	readout_thread_
		= boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DvxplorerRosDriver::readout, this)));

	// camera info handling
	ros::NodeHandle nh_ns(ns);
	camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(nh_ns, device_id_));

	// initialize timestamps
	resetTimestamps();
}

void DvxplorerRosDriver::onDisconnectUSB(void *driver) {
	ROS_ERROR("USB connection lost with DVS !");
	static_cast<dvxplorer_ros_driver::DvxplorerRosDriver *>(driver)->caerConnect();
}

void DvxplorerRosDriver::resetTimestamps() {
	caerDeviceConfigSet(dvxplorer_handle_, DVX_MUX, DVX_MUX_TIMESTAMP_RESET, 1);
	reset_time_ = ros::Time::now();

	ROS_INFO("Reset timestamps on %s to %.9f.", device_id_.c_str(), reset_time_.toSec());

	// if master, publish reset time to slaves
	if (master_) {
		std_msgs::Time reset_msg;
		reset_msg.data = reset_time_;
		reset_pub_.publish(reset_msg);
	}
}

void DvxplorerRosDriver::resetTimestampsCallback(const std_msgs::Time::ConstPtr &msg) {
	// if slave, only adjust offset time
	if (!dvxplorer_info_.deviceIsMaster) {
		ROS_INFO("Adapting reset time of master on slave %s.", device_id_.c_str());
		reset_time_ = msg->data;
	}
	// if master, or not single camera configuration, just reset timestamps
	else {
		resetTimestamps();
	}
}

void DvxplorerRosDriver::imuCalibrationCallback(const std_msgs::Empty::ConstPtr &msg) {
	ROS_INFO("Starting IMU calibration with %d samples...", (int) imu_calibration_sample_size_);
	imu_calibration_running_ = true;
	imu_calibration_samples_.clear();
}

void DvxplorerRosDriver::updateImuBias() {
	bias.linear_acceleration.x = 0.0;
	bias.linear_acceleration.y = 0.0;
	bias.linear_acceleration.z = 0.0;
	bias.angular_velocity.x    = 0.0;
	bias.angular_velocity.y    = 0.0;
	bias.angular_velocity.z    = 0.0;

	for (const auto &m : imu_calibration_samples_) {
		bias.linear_acceleration.x += m.linear_acceleration.x;
		bias.linear_acceleration.y += m.linear_acceleration.y;
		bias.linear_acceleration.z += m.linear_acceleration.z;
		bias.angular_velocity.x += m.angular_velocity.x;
		bias.angular_velocity.y += m.angular_velocity.y;
		bias.angular_velocity.z += m.angular_velocity.z;
	}

	bias.linear_acceleration.x /= (double) imu_calibration_samples_.size();
	bias.linear_acceleration.y /= (double) imu_calibration_samples_.size();
	bias.linear_acceleration.z /= (double) imu_calibration_samples_.size();
	bias.linear_acceleration.z -= STANDARD_GRAVITY * sgn(bias.linear_acceleration.z);

	bias.angular_velocity.x /= (double) imu_calibration_samples_.size();
	bias.angular_velocity.y /= (double) imu_calibration_samples_.size();
	bias.angular_velocity.z /= (double) imu_calibration_samples_.size();

	ROS_INFO("IMU calibration done.");
	ROS_INFO("Acceleration biases: %1.5f %1.5f %1.5f [m/s^2]", bias.linear_acceleration.x, bias.linear_acceleration.y,
		bias.linear_acceleration.z);
	ROS_INFO("Gyroscope biases: %1.5f %1.5f %1.5f [rad/s]", bias.angular_velocity.x, bias.angular_velocity.y,
		bias.angular_velocity.z);
}

void DvxplorerRosDriver::resetTimerCallback(const ros::TimerEvent &te) {
	timestamp_reset_timer_.stop();
	resetTimestamps();
}

void DvxplorerRosDriver::callback(dvxplorer_ros_driver::DVXplorer_ROS_DriverConfig &config, uint32_t level) {
	// All changes have 'level' set.
	if (level == 0) {
		return;
	}

	// DVS control.
	if (level & (0x01 << 0)) {
		caerDeviceConfigSet(dvxplorer_handle_, DVX_DVS, DVX_DVS_RUN, config.dvs_enabled);
		caerDeviceConfigSet(dvxplorer_handle_, DVX_DVS_CHIP_BIAS, DVX_DVS_CHIP_BIAS_SIMPLE, config.bias_sensitivity);
	}

	// Subsampling.
	if (level & (0x01 << 1)) {
		caerDeviceConfigSet(dvxplorer_handle_, DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_ENABLE, config.subsample_enable);
		caerDeviceConfigSet(
			dvxplorer_handle_, DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_HORIZONTAL, config.subsample_horizontal);
		caerDeviceConfigSet(
			dvxplorer_handle_, DVX_DVS_CHIP, DVX_DVS_CHIP_SUBSAMPLE_VERTICAL, config.subsample_vertical);
	}

	// Polarity control.
	if (level & (0x01 << 2)) {
		caerDeviceConfigSet(dvxplorer_handle_, DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_FLATTEN, config.polarity_flatten);
		caerDeviceConfigSet(dvxplorer_handle_, DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_ON_ONLY, config.polarity_on_only);
		caerDeviceConfigSet(dvxplorer_handle_, DVX_DVS_CHIP, DVX_DVS_CHIP_EVENT_OFF_ONLY, config.polarity_off_only);
	}

	// DVS Region Of Interest.
	if (level & (0x01 << 3)) {
		caerDeviceConfigSet(dvxplorer_handle_, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_ENABLE, config.roi_enabled);
		caerDeviceConfigSet(
			dvxplorer_handle_, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_X_START_ADDRESS, config.roi_start_column);
		caerDeviceConfigSet(
			dvxplorer_handle_, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_X_END_ADDRESS, config.roi_end_column);
		caerDeviceConfigSet(
			dvxplorer_handle_, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_Y_START_ADDRESS, config.roi_start_row);
		caerDeviceConfigSet(
			dvxplorer_handle_, DVX_DVS_CHIP_CROPPER, DVX_DVS_CHIP_CROPPER_Y_END_ADDRESS, config.roi_end_row);
	}

	// Inertial Measurement Unit.
	if (level & (0x01 << 4)) {
		caerDeviceConfigSet(dvxplorer_handle_, DVX_IMU, DVX_IMU_RUN_ACCELEROMETER, config.imu_enabled);
		caerDeviceConfigSet(dvxplorer_handle_, DVX_IMU, DVX_IMU_RUN_GYROSCOPE, config.imu_enabled);

		caerDeviceConfigSet(dvxplorer_handle_, DVX_IMU, DVX_IMU_ACCEL_RANGE, config.imu_acc_scale);
		caerDeviceConfigSet(dvxplorer_handle_, DVX_IMU, DVX_IMU_GYRO_RANGE, config.imu_gyro_scale);
	}

	// Streaming rate changes.
	if (level & (0x01 << 5)) {
		if (config.streaming_rate > 0) {
			delta_ = boost::posix_time::microseconds(long(1e6 / config.streaming_rate));
		}

		streaming_rate_ = config.streaming_rate;
		max_events_     = config.max_events;
	}
}

void DvxplorerRosDriver::readout() {
	// std::vector<dvs::Event> events;

	caerDeviceConfigSet(dvxplorer_handle_, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
	caerDeviceDataStart(dvxplorer_handle_, nullptr, nullptr, nullptr, &DvxplorerRosDriver::onDisconnectUSB, this);

	boost::posix_time::ptime next_send_time = boost::posix_time::microsec_clock::local_time();

	dvs_msgs::EventArrayPtr event_array_msg;

	while (running_) {
		try {
			caerEventPacketContainer packetContainer = caerDeviceDataGet(dvxplorer_handle_);
			if (packetContainer == nullptr) {
				continue; // Skip if nothing there.
			}

			const int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);

			for (int32_t i = 0; i < packetNum; i++) {
				caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, i);
				if (packetHeader == nullptr) {
					continue; // Skip if nothing there.
				}

				const int type = caerEventPacketHeaderGetEventType(packetHeader);

				// Packet 0 is always the special events packet for DVS128, while packet is the polarity events packet.
				if (type == POLARITY_EVENT) {
					if (!event_array_msg) {
						event_array_msg         = dvs_msgs::EventArrayPtr(new dvs_msgs::EventArray());
						event_array_msg->height = dvxplorer_info_.dvsSizeY;
						event_array_msg->width  = dvxplorer_info_.dvsSizeX;
					}

					caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;

					const int numEvents = caerEventPacketHeaderGetEventNumber(packetHeader);
					for (int j = 0; j < numEvents; j++) {
						// Get full timestamp and addresses of first event.
						caerPolarityEvent event = caerPolarityEventPacketGetEvent(polarity, j);

						dvs_msgs::Event e;
						e.x  = caerPolarityEventGetX(event);
						e.y  = caerPolarityEventGetY(event);
						e.ts = reset_time_
							   + ros::Duration().fromNSec(caerPolarityEventGetTimestamp64(event, polarity) * 1000);
						e.polarity = caerPolarityEventGetPolarity(event);

						if (j == 0) {
							event_array_msg->header.stamp = e.ts;
						}

						event_array_msg->events.push_back(e);
					}

					int streaming_rate = streaming_rate_;
					int max_events     = max_events_;

					// throttle event messages
					if ((boost::posix_time::microsec_clock::local_time() > next_send_time) || (streaming_rate == 0)
						|| ((max_events != 0) && (event_array_msg->events.size() > max_events))) {
						event_array_pub_.publish(event_array_msg);

						if (streaming_rate > 0) {
							next_send_time += delta_;
						}

						if ((max_events != 0) && (event_array_msg->events.size() > max_events)) {
							next_send_time = boost::posix_time::microsec_clock::local_time() + delta_;
						}

						event_array_msg.reset();
					}

					if (camera_info_manager_->isCalibrated()) {
						sensor_msgs::CameraInfoPtr camera_info_msg(
							new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
						camera_info_pub_.publish(camera_info_msg);
					}
				}
				else if (type == IMU6_EVENT) {
					caerIMU6EventPacket imu = (caerIMU6EventPacket) packetHeader;

					const int numEvents = caerEventPacketHeaderGetEventNumber(packetHeader);

					for (int j = 0; j < numEvents; j++) {
						caerIMU6Event event = caerIMU6EventPacketGetEvent(imu, j);

						sensor_msgs::Imu msg;

						// convert from g's to m/s^2 and align axes with camera frame
						msg.linear_acceleration.x = -caerIMU6EventGetAccelX(event) * STANDARD_GRAVITY;
						msg.linear_acceleration.y = caerIMU6EventGetAccelY(event) * STANDARD_GRAVITY;
						msg.linear_acceleration.z = -caerIMU6EventGetAccelZ(event) * STANDARD_GRAVITY;
						// convert from deg/s to rad/s and align axes with camera frame
						msg.angular_velocity.x = -caerIMU6EventGetGyroX(event) / 180.0 * M_PI;
						msg.angular_velocity.y = caerIMU6EventGetGyroY(event) / 180.0 * M_PI;
						msg.angular_velocity.z = -caerIMU6EventGetGyroZ(event) / 180.0 * M_PI;

						// no orientation estimate: http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
						msg.orientation_covariance[0] = -1.0;

						// time
						msg.header.stamp
							= reset_time_ + ros::Duration().fromNSec(caerIMU6EventGetTimestamp64(event, imu) * 1000);

						// frame
						msg.header.frame_id = "base_link";

						// IMU calibration
						if (imu_calibration_running_) {
							if (imu_calibration_samples_.size() < imu_calibration_sample_size_) {
								imu_calibration_samples_.push_back(msg);
							}
							else {
								imu_calibration_running_ = false;
								updateImuBias();
							}
						}

						// bias correction
						msg.linear_acceleration.x -= bias.linear_acceleration.x;
						msg.linear_acceleration.y -= bias.linear_acceleration.y;
						msg.linear_acceleration.z -= bias.linear_acceleration.z;
						msg.angular_velocity.x -= bias.angular_velocity.x;
						msg.angular_velocity.y -= bias.angular_velocity.y;
						msg.angular_velocity.z -= bias.angular_velocity.z;

						imu_pub_.publish(msg);
					}
				}
			}

			caerEventPacketContainerFree(packetContainer);

			ros::spinOnce();
		}
		catch (boost::thread_interrupted &) {
			return;
		}
	}

	caerDeviceDataStop(dvxplorer_handle_);
}

} // namespace dvxplorer_ros_driver

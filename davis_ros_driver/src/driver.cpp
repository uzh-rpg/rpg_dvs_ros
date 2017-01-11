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

#include "davis_ros_driver/driver.h"

//DAVIS Bias types
#define CF_N_TYPE(COARSE, FINE) (struct caer_bias_coarsefine) \
        { .coarseValue = (uint8_t)(COARSE), .fineValue = (uint8_t)(FINE), \
        .enabled = true, .sexN = true, \
        .typeNormal = true, .currentLevelNormal = true }

#define CF_P_TYPE(COARSE, FINE) (struct caer_bias_coarsefine) \
        { .coarseValue = (uint8_t)(COARSE), .fineValue = (uint8_t)(FINE), \
        .enabled = true, .sexN = false, \
        .typeNormal = true, .currentLevelNormal = true }


namespace davis_ros_driver {

DavisRosDriver::DavisRosDriver(ros::NodeHandle & nh, ros::NodeHandle nh_private) :
    nh_(nh),
    parameter_update_required_(false),
    parameter_bias_update_required_(false),
    imu_calibration_running_(false)
{

  // load parameters
  std::string dvs_serial_number;
  nh_private.param<std::string>("serial_number", dvs_serial_number, "");
  bool master;
  nh_private.param<bool>("master", master, true);
  double reset_timestamps_delay;
  nh_private.param<double>("reset_timestamps_delay", reset_timestamps_delay, -1.0);
  nh_private.param<int>("imu_calibration_sample_size", imu_calibration_sample_size_, 1000);

  // initialize bias
  bias.linear_acceleration.x = 0.0;
  bias.linear_acceleration.y = 0.0;
  bias.linear_acceleration.z = 0.0;
  bias.angular_velocity.x = 0.0;
  bias.angular_velocity.y = 0.0;
  bias.angular_velocity.z = 0.0;


  // set namespace
  ns = ros::this_node::getNamespace();
  if (ns == "/")
    ns = "/dvs";

  event_array_pub_ = nh_.advertise<dvs_msgs::EventArray>(ns + "/events", 10);
  camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(ns + "/camera_info", 1);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(ns + "/imu", 10);
  image_pub_ = nh_.advertise<sensor_msgs::Image>(ns + "/image_raw", 1);

  caerConnect();
  current_config_.streaming_rate = 30;
  delta_ = boost::posix_time::microseconds(1e6/current_config_.streaming_rate);

  reset_sub_ = nh_.subscribe((ns + "/reset_timestamps").c_str(), 1, &DavisRosDriver::resetTimestampsCallback, this);
  imu_calibration_sub_ = nh_.subscribe((ns + "/calibrate_imu").c_str(), 1, &DavisRosDriver::imuCalibrationCallback, this);
  snapshot_sub_ = nh_.subscribe((ns + "/trigger_snapshot").c_str(), 1, &DavisRosDriver::snapshotCallback, this);

  // Dynamic reconfigure
  dynamic_reconfigure_callback_ = boost::bind(&DavisRosDriver::callback, this, _1, _2);
  server_.reset(new dynamic_reconfigure::Server<davis_ros_driver::DAVIS_ROS_DriverConfig>(nh_private));
  server_->setCallback(dynamic_reconfigure_callback_);

  // start timer to reset timestamps for synchronization
  if (reset_timestamps_delay > 0.0)
  {
    timestamp_reset_timer_ = nh_.createTimer(ros::Duration(reset_timestamps_delay), &DavisRosDriver::resetTimerCallback, this);
    ROS_INFO("Started timer to reset timestamps on master DVS for synchronization (delay=%3.2fs).", reset_timestamps_delay);
  }
}

DavisRosDriver::~DavisRosDriver()
{
  if (running_)
  {
    ROS_INFO("shutting down threads");
    running_ = false;
    parameter_thread_->join();
    readout_thread_->join();
    ROS_INFO("threads stopped");

    caerDeviceClose(&davis_handle_);
  }
}

void DavisRosDriver::caerConnect()
{

  // start driver
  bool dvs_running = false;
  while (!dvs_running)
  {
    //driver_ = new dvs::DvsDriver(dvs_serial_number, master);
    davis_handle_ = caerDeviceOpen(1, CAER_DEVICE_DAVIS_FX2, 0, 0, NULL);

    //dvs_running = driver_->isDeviceRunning();
    dvs_running = !(davis_handle_ == NULL);

    if (!dvs_running)
    {
      ROS_WARN("Could not find DVS. Will retry every second.");
      ros::Duration(1.0).sleep();
      ros::spinOnce();
    }

    if (!ros::ok())
    {
      return;
    }
  }

  davis_info_ = caerDavisInfoGet(davis_handle_);
  device_id_ = "DAVIS-" + std::string(davis_info_.deviceString).substr(18, 8);

  ROS_INFO("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Logic: %d.\n", davis_info_.deviceString,
           davis_info_.deviceID, davis_info_.deviceIsMaster, davis_info_.dvsSizeX, davis_info_.dvsSizeY,
           davis_info_.logicVersion);

  // Send the default configuration before using the device.
  // No configuration is sent automatically!
  caerDeviceSendDefaultConfig(davis_handle_);

  // Re-send params from param server if not first connection
  parameter_bias_update_required_ = true;
  parameter_update_required_ = true;

  // camera info handling
  ros::NodeHandle nh_ns(ns);
  if(camera_info_manager_){
    delete camera_info_manager_;
  }

  camera_info_manager_ = new camera_info_manager::CameraInfoManager(nh_ns, device_id_);

  // initialize timestamps
  resetTimestamps();
  reset_time_ = ros::Time::now();

  // spawn threads
  running_ = true;
  parameter_thread_ = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&DavisRosDriver::changeDvsParameters, this)));
  readout_thread_ = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&DavisRosDriver::readout, this)));

}

void DavisRosDriver::onDisconnectUSB(void* driver){
  ROS_ERROR("USB connection lost with DVS !");
  static_cast<davis_ros_driver::DavisRosDriver*>(driver)->caerConnect();
}

void DavisRosDriver::resetTimestamps()
{
  ROS_INFO("Reset timestamps on %s", device_id_.c_str());
  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, 1);
}

void DavisRosDriver::resetTimestampsCallback(const std_msgs::Empty::ConstPtr& msg)
{
  resetTimestamps();
}

void DavisRosDriver::imuCalibrationCallback(const std_msgs::Empty::ConstPtr &msg)
{
  ROS_INFO("Starting IMU calibration with %d samples...", (int) imu_calibration_sample_size_);
  imu_calibration_running_ = true;
  imu_calibration_samples_.clear();
}

void DavisRosDriver::updateImuBias()
{
  bias.linear_acceleration.x = 0.0;
  bias.linear_acceleration.y = 0.0;
  bias.linear_acceleration.z = 0.0;
  bias.angular_velocity.x = 0.0;
  bias.angular_velocity.y = 0.0;
  bias.angular_velocity.z = 0.0;

  for (auto m : imu_calibration_samples_)
  {
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
  ROS_INFO("Acceleration biases: %1.5f %1.5f %1.5f [m/s^2]", bias.linear_acceleration.x,
           bias.linear_acceleration.y, bias.linear_acceleration.z);
  ROS_INFO("Gyroscope biases: %1.5f %1.5f %1.5f [rad/s]", bias.angular_velocity.x,
           bias.angular_velocity.y, bias.angular_velocity.z);
}

void DavisRosDriver::snapshotCallback(const std_msgs::Empty::ConstPtr& msg)
{
  // only trigger frame readout when APS not running
  if (!current_config_.aps_enabled)
  {
    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_SNAPSHOT, 1);
  }
  else
  {
    ROS_WARN("Snapshot will only be taken when APS is not enabled.");
  }
}

void DavisRosDriver::resetTimerCallback(const ros::TimerEvent& te)
{
  resetTimestamps();
  timestamp_reset_timer_.stop();
}

void DavisRosDriver::changeDvsParameters()
{
  while(running_)
  {
    try
    {
      if (parameter_update_required_)
      {
        parameter_update_required_ = false;
        caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, current_config_.exposure);
        caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_DELAY, current_config_.frame_delay);

        caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, current_config_.aps_enabled);
        caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, current_config_.dvs_enabled);
        caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN, current_config_.imu_enabled);

        if (current_config_.imu_gyro_scale >= 0 && current_config_.imu_gyro_scale <= 3)
          caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE, current_config_.imu_gyro_scale);

        if (current_config_.imu_acc_scale >= 0 && current_config_.imu_acc_scale <= 3)
          caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE, current_config_.imu_acc_scale);
      }

      // BIAS changes for DAVIS240
      if (parameter_bias_update_required_)
      {
        parameter_bias_update_required_ = false;
        caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP,
            caerBiasCoarseFineGenerate(CF_P_TYPE(current_config_.PrBp_coarse, current_config_.PrBp_fine)));
        caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP,
            caerBiasCoarseFineGenerate(CF_P_TYPE(current_config_.PrSFBp_coarse, current_config_.PrSFBp_fine)));

        caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_DIFFBN,
            caerBiasCoarseFineGenerate(CF_N_TYPE(current_config_.DiffBn_coarse, current_config_.DiffBn_fine)));
        caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_ONBN,
            caerBiasCoarseFineGenerate(CF_N_TYPE(current_config_.ONBn_coarse, current_config_.ONBn_fine)));
        caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_OFFBN,
            caerBiasCoarseFineGenerate(CF_N_TYPE(current_config_.OFFBn_coarse, current_config_.OFFBn_fine)));

        caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_REFRBP,
            caerBiasCoarseFineGenerate(CF_P_TYPE(current_config_.RefrBp_coarse, current_config_.RefrBp_fine)));

      }

      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
    catch(boost::thread_interrupted&)
    {
      return;
    }
  }
}

void DavisRosDriver::callback(davis_ros_driver::DAVIS_ROS_DriverConfig &config, uint32_t level)
{
  // did any DVS bias setting change?
   if (current_config_.exposure != config.exposure || current_config_.frame_delay != config.frame_delay ||
       current_config_.aps_enabled != config.aps_enabled || current_config_.dvs_enabled != config.dvs_enabled ||
       current_config_.imu_enabled != config.imu_enabled || current_config_.imu_acc_scale != config.imu_acc_scale ||
       current_config_.imu_gyro_scale != config.imu_gyro_scale || current_config_.max_events != config.max_events)
   {
     current_config_.exposure = config.exposure;
     current_config_.frame_delay = config.frame_delay;

     current_config_.aps_enabled = config.aps_enabled;
     current_config_.dvs_enabled = config.dvs_enabled;
     current_config_.imu_enabled = config.imu_enabled;

     current_config_.imu_acc_scale = config.imu_acc_scale;
     current_config_.imu_gyro_scale = config.imu_gyro_scale;

     current_config_.max_events = config.max_events;

     parameter_update_required_ = true;
   }

   if (level & 1)
   {
     parameter_bias_update_required_ = true;
     current_config_.PrBp_coarse = config.PrBp_coarse;
     current_config_.PrBp_fine = config.PrBp_fine;
     current_config_.PrSFBp_coarse = config.PrSFBp_coarse;
     current_config_.PrSFBp_fine = config.PrSFBp_fine;
     current_config_.DiffBn_coarse = config.DiffBn_coarse;
     current_config_.DiffBn_fine = config.DiffBn_fine;
     current_config_.ONBn_coarse = config.ONBn_coarse;
     current_config_.ONBn_fine = config.ONBn_fine;
     current_config_.OFFBn_coarse = config.OFFBn_coarse;
     current_config_.OFFBn_fine = config.OFFBn_fine;
     current_config_.RefrBp_coarse = config.RefrBp_coarse;
     current_config_.RefrBp_fine = config.RefrBp_fine;
   }

   // change streaming rate, if necessary
   if (current_config_.streaming_rate != config.streaming_rate)
   {
     current_config_.streaming_rate = config.streaming_rate;
     if (current_config_.streaming_rate > 0)
     {
       delta_ = boost::posix_time::microseconds(1e6/current_config_.streaming_rate);
     }
   }
}

void DavisRosDriver::readout()
{

  //std::vector<dvs::Event> events;

  caerDeviceDataStart(davis_handle_, NULL, NULL, NULL, &DavisRosDriver::onDisconnectUSB, this);
  caerDeviceConfigSet(davis_handle_, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

  boost::posix_time::ptime next_send_time = boost::posix_time::microsec_clock::local_time();

  dvs_msgs::EventArrayPtr event_array_msg;

  while (running_)
  {
    try
    {
      caerEventPacketContainer packetContainer = caerDeviceDataGet(davis_handle_);
      if (packetContainer == NULL)
      {
        continue; // Skip if nothing there.
      }

      int32_t packetNum = caerEventPacketContainerGetEventPacketsNumber(packetContainer);

      for (int32_t i = 0; i < packetNum; i++)
      {
        caerEventPacketHeader packetHeader = caerEventPacketContainerGetEventPacket(packetContainer, i);
        if (packetHeader == NULL)
        {
          continue; // Skip if nothing there.
        }

        // Packet 0 is always the special events packet for DVS128, while packet is the polarity events packet.
        if (i == POLARITY_EVENT)
        {
          if (!event_array_msg)
          {
            event_array_msg = dvs_msgs::EventArrayPtr(new dvs_msgs::EventArray());
            event_array_msg->height = davis_info_.dvsSizeY;
            event_array_msg->width = davis_info_.dvsSizeX;
          }

          caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;

          const int numEvents = caerEventPacketHeaderGetEventNumber(packetHeader);
          for (int j = 0; j < numEvents; j++)
          {
            // Get full timestamp and addresses of first event.
            caerPolarityEvent event = caerPolarityEventPacketGetEvent(polarity, j);

            dvs_msgs::Event e;
            e.x = caerPolarityEventGetX(event);
            e.y = caerPolarityEventGetY(event);
            e.ts = reset_time_ + ros::Duration(caerPolarityEventGetTimestamp64(event, polarity) / 1.e6);
            e.polarity = caerPolarityEventGetPolarity(event);

            event_array_msg->events.push_back(e);
          }

          // throttle event messages
          if (boost::posix_time::microsec_clock::local_time() > next_send_time ||
              current_config_.streaming_rate == 0 ||
              (current_config_.max_events != 0 && event_array_msg->events.size() > current_config_.max_events)
             )
          {
            event_array_pub_.publish(event_array_msg);

            if (current_config_.streaming_rate > 0)
              next_send_time += delta_;
            if (current_config_.max_events != 0 && event_array_msg->events.size() > current_config_.max_events)
              next_send_time = boost::posix_time::microsec_clock::local_time() + delta_;

            event_array_msg.reset();
          }

          if (camera_info_manager_->isCalibrated())
          {
            sensor_msgs::CameraInfoPtr camera_info_msg(new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
            camera_info_pub_.publish(camera_info_msg);
          }
        }
        else if (i == IMU6_EVENT)
        {
          caerIMU6EventPacket imu = (caerIMU6EventPacket) packetHeader;

          const int numEvents = caerEventPacketHeaderGetEventNumber(packetHeader);

          for (int j = 0; j < numEvents; j++)
          {
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
            msg.header.stamp = reset_time_ + ros::Duration(caerIMU6EventGetTimestamp64(event, imu) / 1.e6);

            // frame
            msg.header.frame_id = "base_link";

            // IMU calibration
            if (imu_calibration_running_)
            {
              if (imu_calibration_samples_.size() < imu_calibration_sample_size_)
              {
                imu_calibration_samples_.push_back(msg);
              }
              else
              {
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

        else if (i == FRAME_EVENT)
        {
          caerFrameEventPacket frame = (caerFrameEventPacket) packetHeader;
          caerFrameEvent event = caerFrameEventPacketGetEvent(frame, 0);

          uint16_t* image = caerFrameEventGetPixelArrayUnsafe(event);

          sensor_msgs::Image msg;

          // meta information
          msg.encoding = "mono8";
          msg.width = davis_info_.apsSizeX;
          msg.height = davis_info_.apsSizeY;
          msg.step = davis_info_.apsSizeX;

          // image data
          const int32_t frame_width = caerFrameEventGetLengthX(event);
          const int32_t frame_height= caerFrameEventGetLengthY(event);

          for (int img_y=0; img_y<frame_height; img_y++)
          {
            for (int img_x=0; img_x<frame_width; img_x++)
            {
              const uint16_t value = image[img_y*frame_width + img_x];
              //msg.data.push_back(value & 0xff);
              msg.data.push_back(value >> 8);
            }
          }

          // time
          msg.header.stamp = reset_time_ + ros::Duration(caerFrameEventGetTimestamp64(event, frame) / 1.e6);

          image_pub_.publish(msg);
        }
      }

      caerEventPacketContainerFree(packetContainer);

      ros::spinOnce();
    }
    catch (boost::thread_interrupted&)
    {
      return;
    }
  }

  caerDeviceDataStop(davis_handle_);
}

} // namespace

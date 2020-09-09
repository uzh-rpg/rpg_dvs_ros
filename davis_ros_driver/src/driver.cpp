// This file is part of DVS-ROS - the RPG DVS ROS Package

#include "davis_ros_driver/driver.h"
#include "davis_ros_driver/driver_utils.h"
#include <std_msgs/Int32.h>

//DAVIS Bias types
#define CF_N_TYPE(COARSE, FINE) (struct caer_bias_coarsefine) \
{ .coarseValue = (uint8_t)(COARSE), .fineValue = (uint8_t)(FINE), \
    .enabled = true, .sexN = true, \
    .typeNormal = true, .currentLevelNormal = true }

#define CF_P_TYPE(COARSE, FINE) (struct caer_bias_coarsefine) \
{ .coarseValue = (uint8_t)(COARSE), .fineValue = (uint8_t)(FINE), \
    .enabled = true, .sexN = false, \
    .typeNormal = true, .currentLevelNormal = true }

#define SHIFTSOURCE(REF, REG, OPMODE) (struct caer_bias_shiftedsource) \
{ .refValue = (uint8_t)(REF), .regValue = (uint8_t)(REG), \
    .operatingMode = (caer_bias_shiftedsource_operating_mode)(OPMODE), .voltageLevel = (caer_bias_shiftedsource_voltage_level)(SPLIT_GATE) }

#define VDAC(VOLT, CURR) (struct caer_bias_vdac) \
{ .voltageValue = (uint8_t)(VOLT), .currentValue = (uint8_t)(CURR) }


namespace davis_ros_driver {

DavisRosDriver::DavisRosDriver(ros::NodeHandle & nh, ros::NodeHandle nh_private) :
    nh_(nh),
    parameter_update_required_(false),
    parameter_bias_update_required_(false),
    imu_calibration_running_(false)
{
  // load parameters
  nh_private.param<std::string>("serial_number", device_id_, "");
  nh_private.param<bool>("master", master_, true);
  double reset_timestamps_delay;
  nh_private.param<double>("reset_timestamps_delay", reset_timestamps_delay, -1.0);
  nh_private.param<int>("imu_calibration_sample_size", imu_calibration_sample_size_, 1000);

  // initialize bias
  nh_private.param<double>("imu_bias/ax", bias.linear_acceleration.x, 0.0);
  nh_private.param<double>("imu_bias/ay", bias.linear_acceleration.y, 0.0);
  nh_private.param<double>("imu_bias/az", bias.linear_acceleration.z, 0.0);
  nh_private.param<double>("imu_bias/wx", bias.angular_velocity.x, 0.0);
  nh_private.param<double>("imu_bias/wy", bias.angular_velocity.y, 0.0);
  nh_private.param<double>("imu_bias/wz", bias.angular_velocity.z, 0.0);

  // set namespace
  ns = ros::this_node::getNamespace();
  if (ns == "/")
    ns = "/dvs";

  event_array_pub_ = nh_.advertise<dvs_msgs::EventArray>(ns + "/events", 10);
  camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(ns + "/camera_info", 1);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(ns + "/imu", 10);
  image_pub_ = nh_.advertise<sensor_msgs::Image>(ns + "/image_raw", 1);
  exposure_pub_ = nh_.advertise<std_msgs::Int32>(ns + "/exposure", 10);

  // reset timestamps is publisher as master, subscriber as slave
  if (master_)
  {
    reset_pub_ = nh_.advertise<std_msgs::Time>((ns + "/reset_timestamps").c_str(), 1);
  }
  else
  {
    reset_sub_ = nh_.subscribe((ns + "/reset_timestamps").c_str(), 1, &DavisRosDriver::resetTimestampsCallback, this);
  }

  // Dynamic reconfigure
  // It is important that the dynamic reconfigure callback is set before caerConnect().
  // The dynamic reconfigure callback will be called directly when registered,
  // which will initialize properly current_config_
  dynamic_reconfigure_callback_ = boost::bind(&DavisRosDriver::callback, this, _1, _2);
  server_.reset(new dynamic_reconfigure::Server<davis_ros_driver::DAVIS_ROS_DriverConfig>(config_mutex, nh_private));
  server_->setCallback(dynamic_reconfigure_callback_);

  caerConnect();
  current_config_.streaming_rate = 30;
  delta_ = boost::posix_time::microseconds(long(1e6/current_config_.streaming_rate));

  imu_calibration_sub_ = nh_.subscribe((ns + "/calibrate_imu").c_str(), 1, &DavisRosDriver::imuCalibrationCallback, this);
  snapshot_sub_ = nh_.subscribe((ns + "/trigger_snapshot").c_str(), 1, &DavisRosDriver::snapshotCallback, this);

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
        caerLog(CAER_LOG_ERROR, "destructor",  "data stop now");
        caerDeviceDataStop(davis_handle_);
        caerDeviceClose(&davis_handle_);
    }
}

void DavisRosDriver::dataStop()
{
    caerLog(CAER_LOG_INFO, "Exiting from driver node",  "executing data stop");
    ROS_INFO("Exiting from driver node, executing data stop");
    caerDeviceDataStop(davis_handle_);
    caerDeviceClose(&davis_handle_);
}

void DavisRosDriver::caerConnect()
{

  // start driver
  bool device_is_running = false;
  while (!device_is_running)
  {
    const char* serial_number_restrict = (device_id_ == "") ? NULL : device_id_.c_str();

    if(serial_number_restrict)
    {
      ROS_WARN("Requested serial number: %s", device_id_.c_str());
    }

    davis_handle_ = caerDeviceOpen(1, CAER_DEVICE_DAVIS, 0, 0, serial_number_restrict);

    // was opening successful?
    device_is_running = !(davis_handle_ == NULL);

    if (!device_is_running)
    {
      ROS_WARN("Could not find DAVIS. Will retry every second.");
      ros::Duration(1.0).sleep();
      ros::spinOnce();
    }

    if (!ros::ok())
    {
      return;
    }
  }

  davis_info_ = caerDavisInfoGet(davis_handle_);
  device_id_ = "DAVIS-" + std::string(davis_info_.deviceString).substr(14, 8);

  ROS_INFO("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Logic: %d.\n", davis_info_.deviceString,
           davis_info_.deviceID, davis_info_.deviceIsMaster, davis_info_.dvsSizeX, davis_info_.dvsSizeY,
           davis_info_.firmwareVersion);

  if (master_ && !davis_info_.deviceIsMaster)
  {
    ROS_WARN("Device %s should be master, but is not!", device_id_.c_str());
  }

  // Send the default configuration before using the device.
  // No configuration is sent automatically!
  caerDeviceSendDefaultConfig(davis_handle_);

  // In case autoexposure is enabled, initialize the exposure time with the exposure value
  // from the parameter server
  if(current_config_.autoexposure_enabled)
  {
    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, current_config_.exposure);
  }

  // Re-send params from param server if not first connection
  parameter_bias_update_required_ = true;
  parameter_update_required_ = true;

  // camera info handling
  ros::NodeHandle nh_ns(ns);
  camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(nh_ns, device_id_));

  // spawn threads
  running_ = true;
  parameter_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DavisRosDriver::changeDvsParameters, this)));
  readout_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DavisRosDriver::readout, this)));

  // wait for driver to be ready
  ros::Duration(0.6).sleep();

  // initialize timestamps
  resetTimestamps();
}

void DavisRosDriver::onDisconnectUSB(void* driver)
{
    ROS_ERROR("USB connection lost with DVS !");
    static_cast<davis_ros_driver::DavisRosDriver*>(driver)->caerConnect();
}

void DavisRosDriver::resetTimestamps()
{
  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, 1);
  reset_time_ = ros::Time::now();

  ROS_INFO("Reset timestamps on %s to %.9f.", device_id_.c_str(), reset_time_.toSec());

  // if master, publish reset time to slaves
  if (master_)
  {
    std_msgs::Time reset_msg;
    reset_msg.data = reset_time_;
    reset_pub_.publish(reset_msg);
  }
}

void DavisRosDriver::resetTimestampsCallback(const std_msgs::Time::ConstPtr& msg)
{
  // if slave, only adjust offset time
  if (!davis_info_.deviceIsMaster)
  {
    ROS_INFO("Adapting reset time of master on slave %s.", device_id_.c_str());
    reset_time_ = msg->data;
  }
  // if master, or not single camera configuration, just reset timestamps
  else
  {
    resetTimestamps();
  }
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
  timestamp_reset_timer_.stop();
  resetTimestamps();
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

                if(!current_config_.autoexposure_enabled) {
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, current_config_.exposure);
                }
                
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_MODE, current_config_.frame_mode);
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_FRAME_INTERVAL, current_config_.frame_interval);

                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, current_config_.aps_enabled);
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, current_config_.dvs_enabled);
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_ACCELEROMETER, current_config_.imu_enabled);
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_GYROSCOPE, current_config_.imu_enabled);
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_TEMPERATURE, current_config_.imu_enabled);

                if (current_config_.imu_gyro_scale >= 0 && current_config_.imu_gyro_scale <= 3)
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_GYRO_FULL_SCALE, current_config_.imu_gyro_scale);

                if (current_config_.imu_acc_scale >= 0 && current_config_.imu_acc_scale <= 3)
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_ACCEL_FULL_SCALE, current_config_.imu_acc_scale);

                if (current_config_.imu_low_pass_filter >= 0 && current_config_.imu_low_pass_filter <= 6)
                {
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_DIGITAL_LOW_PASS_FILTER, current_config_.imu_low_pass_filter);

                    if(current_config_.imu_low_pass_filter == 0)
                    {
                      // When the low pass filter is disabled, the output frequency of IMU events
                      // is raised to 8KHz. To keep it to 1 kHz, we use the sample rate divider
                      // (setting its value to 7 divides the frequency by 8).
                      caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER, 7);
                    }
                    else
                    {
                      // When the low pass filter is enabled, the gyroscope output rate is set to 1 kHz,
                      // so we should not use the sample rate divider, in order to keep the IMU output rate to 1 kHz.
                      caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_SAMPLE_RATE_DIVIDER, 0);
                    }
                }
            }
            /*
       * Set Sensor-dependent Biases
       */
            if (davis_info_.chipID == DAVIS_CHIP_DAVIS346B)
            {
                // VDAC
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_APSOVERFLOWLEVEL,
                                    caerBiasVDACGenerate(VDAC(27,6)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_APSCAS,
                                    caerBiasVDACGenerate(VDAC(21,6)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCREFHIGH,
                                    caerBiasVDACGenerate(VDAC(current_config_.ADC_RefHigh_volt, current_config_.ADC_RefHigh_curr)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCREFLOW,
                                    caerBiasVDACGenerate(VDAC(current_config_.ADC_RefLow_volt, current_config_.ADC_RefLow_curr)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCTESTVOLTAGE,
                                    caerBiasVDACGenerate(VDAC(21,7)));
                // CF Biases
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_LOCALBUFBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(5, 164)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PADFOLLBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(7, 215)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_DIFFBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(current_config_.DiffBn_coarse, current_config_.DiffBn_fine)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ONBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(current_config_.ONBn_coarse, current_config_.ONBn_fine)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_OFFBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(current_config_.OFFBn_coarse, current_config_.OFFBn_fine)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PIXINVBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(5, 129)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PRBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(current_config_.PrBp_coarse, current_config_.PrBp_fine)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_PRSFBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(current_config_.PrSFBp_coarse, current_config_.PrSFBp_fine)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_REFRBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(current_config_.RefrBp_coarse, current_config_.RefrBp_fine)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_READOUTBUFBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(6, 20)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_APSROSFBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(6, 219)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_ADCCOMPBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(5, 20)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_COLSELLOWBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(0, 1)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_DACBUFBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(6, 60)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_LCOLTIMEOUTBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(5, 49)));;
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_AEPDBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(6, 91)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_AEPUXBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(4, 80)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_AEPUYBP,
                                    caerBiasCoarseFineGenerate(CF_P_TYPE(7, 152)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_IFREFRBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_IFTHRBN,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(5, 255)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_BIASBUFFER,
                                    caerBiasCoarseFineGenerate(CF_N_TYPE(5, 254)));
                // Special Biases
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_SSP,
                                    caerBiasShiftedSourceGenerate(SHIFTSOURCE(1,33,SHIFTED_SOURCE)));
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_BIAS, DAVIS346_CONFIG_BIAS_SSN,
                                    caerBiasShiftedSourceGenerate(SHIFTSOURCE(1,33,SHIFTED_SOURCE)));

                // Hardware filters
                if (davis_info_.dvsHasPixelFilter)
                {
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_AUTO_TRAIN, current_config_.pixel_auto_train);
                  
                  // if using auto train, update the configuration with hardware values
                  if (current_config_.pixel_auto_train)
                  {
                    ROS_INFO("Auto-training hot-pixel filter...");
                    while(current_config_.pixel_auto_train)
                    {
                      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                      caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_AUTO_TRAIN, (uint32_t*)&current_config_.pixel_auto_train);
                    }
                  
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW, (uint32_t*)&current_config_.pixel_0_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN, (uint32_t*)&current_config_.pixel_0_column);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW, (uint32_t*)&current_config_.pixel_1_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN, (uint32_t*)&current_config_.pixel_1_column);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW, (uint32_t*)&current_config_.pixel_2_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN, (uint32_t*)&current_config_.pixel_2_column);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW, (uint32_t*)&current_config_.pixel_3_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN, (uint32_t*)&current_config_.pixel_3_column);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW, (uint32_t*)&current_config_.pixel_4_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN, (uint32_t*)&current_config_.pixel_4_column);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW, (uint32_t*)&current_config_.pixel_5_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN, (uint32_t*)&current_config_.pixel_5_column);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW, (uint32_t*)&current_config_.pixel_6_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN, (uint32_t*)&current_config_.pixel_6_column);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW, (uint32_t*)&current_config_.pixel_7_row);
                    caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN, (uint32_t*)&current_config_.pixel_7_column);
                    
                    boost::recursive_mutex::scoped_lock lock(config_mutex);
                    server_->updateConfig(current_config_);
                    lock.unlock();
                    ROS_INFO("Done auto-training hot-pixel filter.");
                  }
                  else // apply current configuration to hardware
                  {
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_ROW, current_config_.pixel_0_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_0_COLUMN, current_config_.pixel_0_column);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_ROW, current_config_.pixel_1_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_1_COLUMN, current_config_.pixel_1_column);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_ROW, current_config_.pixel_2_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_2_COLUMN, current_config_.pixel_2_column);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_ROW, current_config_.pixel_3_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_3_COLUMN, current_config_.pixel_3_column);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_ROW, current_config_.pixel_4_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_4_COLUMN, current_config_.pixel_4_column);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_ROW, current_config_.pixel_5_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_5_COLUMN, current_config_.pixel_5_column);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_ROW, current_config_.pixel_6_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_6_COLUMN, current_config_.pixel_6_column);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_ROW, current_config_.pixel_7_row);
                    caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_PIXEL_7_COLUMN, current_config_.pixel_7_column);
                  }
                }
                
                if (davis_info_.dvsHasBackgroundActivityFilter)
                {
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY, current_config_.background_activity_filter_enabled);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_BACKGROUND_ACTIVITY_TIME, current_config_.background_activity_filter_time);

                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD, current_config_.refractory_period_enabled);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_REFRACTORY_PERIOD_TIME, current_config_.refractory_period_time);
                }
                
                if (davis_info_.dvsHasROIFilter)
                {
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_COLUMN, current_config_.roi_start_column);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_START_ROW, current_config_.roi_start_row);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_COLUMN, current_config_.roi_end_column);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_ROI_END_ROW, current_config_.roi_end_row);
                }
                
                if (davis_info_.dvsHasSkipFilter)
                {
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS, current_config_.skip_enabled);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_SKIP_EVENTS_EVERY, current_config_.skip_every);
                }
                
                if (davis_info_.dvsHasPolarityFilter)
                {
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_FLATTEN, current_config_.polarity_flatten);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS, current_config_.polarity_suppress);
                  caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_FILTER_POLARITY_SUPPRESS_TYPE, current_config_.polarity_suppress_type);
                }
                
                // APS region of interest
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_COLUMN_0, current_config_.aps_roi_start_column);
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_START_ROW_0, current_config_.aps_roi_start_row);
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_COLUMN_0, current_config_.aps_roi_end_column);
                caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_END_ROW_0, current_config_.aps_roi_end_row);
            }
            else
            {
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
    if (current_config_.exposure != config.exposure ||
            current_config_.frame_mode != config.frame_mode || current_config_.frame_interval != config.frame_interval ||
            current_config_.autoexposure_enabled != config.autoexposure_enabled || current_config_.autoexposure_gain != config.autoexposure_gain ||
            current_config_.aps_enabled != config.aps_enabled || current_config_.dvs_enabled != config.dvs_enabled ||
            current_config_.imu_enabled != config.imu_enabled || current_config_.imu_acc_scale != config.imu_acc_scale ||
            current_config_.imu_gyro_scale != config.imu_gyro_scale || current_config_.imu_low_pass_filter != config.imu_low_pass_filter ||
            current_config_.max_events != config.max_events || current_config_.autoexposure_desired_intensity != config.autoexposure_desired_intensity)
    {
        current_config_.exposure = config.exposure;
        
        current_config_.frame_mode = config.frame_mode;
        current_config_.frame_interval = config.frame_interval;

        current_config_.autoexposure_enabled = config.autoexposure_enabled;
        current_config_.autoexposure_gain = config.autoexposure_gain;
        current_config_.autoexposure_desired_intensity = config.autoexposure_desired_intensity;

        current_config_.aps_enabled = config.aps_enabled;
        current_config_.dvs_enabled = config.dvs_enabled;
        current_config_.imu_enabled = config.imu_enabled;

        current_config_.imu_acc_scale = config.imu_acc_scale;
        current_config_.imu_gyro_scale = config.imu_gyro_scale;
        current_config_.imu_low_pass_filter = config.imu_low_pass_filter;

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
        current_config_.ADC_RefHigh_volt = config.ADC_RefHigh_volt;
        current_config_.ADC_RefHigh_curr = config.ADC_RefHigh_curr;
        current_config_.ADC_RefLow_volt = config.ADC_RefLow_volt;
        current_config_.ADC_RefLow_curr = config.ADC_RefLow_curr;
        current_config_.background_activity_filter_enabled = config.background_activity_filter_enabled;
        current_config_.background_activity_filter_time = config.background_activity_filter_time;
        current_config_.refractory_period_enabled = config.refractory_period_enabled;
        current_config_.refractory_period_time = config.refractory_period_time;
        current_config_.roi_start_column = config.roi_start_column;
        current_config_.roi_start_row = config.roi_start_row;
        current_config_.roi_end_column = config.roi_end_column;
        current_config_.roi_end_row = config.roi_end_row;
        current_config_.aps_roi_start_column = config.aps_roi_start_column;
        current_config_.aps_roi_start_row = config.aps_roi_start_row;
        current_config_.aps_roi_end_column = config.aps_roi_end_column;
        current_config_.aps_roi_end_row = config.aps_roi_end_row;
        current_config_.skip_enabled = config.skip_enabled;
        current_config_.skip_every = config.skip_every;
        current_config_.polarity_flatten = config.polarity_flatten;
        current_config_.polarity_suppress = config.polarity_suppress;
        current_config_.polarity_suppress_type = config.polarity_suppress_type;
        current_config_.pixel_0_row = config.pixel_0_row;
        current_config_.pixel_0_column = config.pixel_0_column;
        current_config_.pixel_1_row = config.pixel_1_row;
        current_config_.pixel_1_column = config.pixel_1_column;
        current_config_.pixel_2_row = config.pixel_2_row;
        current_config_.pixel_2_column = config.pixel_2_column;
        current_config_.pixel_3_row = config.pixel_3_row;
        current_config_.pixel_3_column = config.pixel_3_column;
        current_config_.pixel_4_row = config.pixel_4_row;
        current_config_.pixel_4_column = config.pixel_4_column;
        current_config_.pixel_5_row = config.pixel_5_row;
        current_config_.pixel_5_column = config.pixel_5_column;
        current_config_.pixel_6_row = config.pixel_6_row;
        current_config_.pixel_6_column = config.pixel_6_column;
        current_config_.pixel_7_row = config.pixel_7_row;
        current_config_.pixel_7_column = config.pixel_7_column;
        current_config_.pixel_auto_train = config.pixel_auto_train;
    }

    // change streaming rate, if necessary
    if (current_config_.streaming_rate != config.streaming_rate)
    {
        current_config_.streaming_rate = config.streaming_rate;
        if (current_config_.streaming_rate > 0)
        {
            delta_ = boost::posix_time::microseconds(long(1e6/current_config_.streaming_rate));
        }
    }
}

void DavisRosDriver::readout()
{

    //std::vector<dvs::Event> events;

    caerDeviceConfigSet(davis_handle_, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
    caerDeviceDataStart(davis_handle_, NULL, NULL, NULL, &DavisRosDriver::onDisconnectUSB, this);

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

                const int type = caerEventPacketHeaderGetEventType(packetHeader);

                // Packet 0 is always the special events packet for DVS128, while packet is the polarity events packet.
                if (type == POLARITY_EVENT)
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
                        e.ts = reset_time_
                                + ros::Duration().fromNSec(caerPolarityEventGetTimestamp64(event, polarity) * 1000);
                        e.polarity = caerPolarityEventGetPolarity(event);

                        if(j == 0)
                        {
                            event_array_msg->header.stamp = e.ts;
                        }

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
                        {
                            next_send_time += delta_;
                        }
                        if (current_config_.max_events != 0 && event_array_msg->events.size() > current_config_.max_events)
                        {
                            next_send_time = boost::posix_time::microsec_clock::local_time() + delta_;
                        }

                        event_array_msg.reset();
                    }

                    if (camera_info_manager_->isCalibrated())
                    {
                        sensor_msgs::CameraInfoPtr camera_info_msg(new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
                        camera_info_pub_.publish(camera_info_msg);
                    }
                }
                else if (type == IMU6_EVENT)
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
                        msg.header.stamp = reset_time_ +
                                ros::Duration().fromNSec(caerIMU6EventGetTimestamp64(event, imu) * 1000);

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
                else if (type == FRAME_EVENT)
                {
                    caerFrameEventPacket frame = (caerFrameEventPacket) packetHeader;
                    caerFrameEvent event = caerFrameEventPacketGetEvent(frame, 0);

                    uint16_t* image = caerFrameEventGetPixelArrayUnsafe(event);

                    sensor_msgs::Image msg;
                    
                    // get image metadata
                    caer_frame_event_color_channels frame_channels = caerFrameEventGetChannelNumber(event);
                    const int32_t frame_width = caerFrameEventGetLengthX(event);
                    const int32_t frame_height = caerFrameEventGetLengthY(event);
                    
                    // set message metadata
                    msg.width = frame_width;
                    msg.height = frame_height;
                    msg.step = frame_width * frame_channels;
                    
                    if (frame_channels==1)
                    {
                      msg.encoding = "mono8";
                    }
                    else if (frame_channels==3)
                    {
                      msg.encoding = "rgb8";
                    }
                    
                    // set message data
                    for (int img_y=0; img_y<frame_height*frame_channels; img_y++)
                    {
                        for (int img_x=0; img_x<frame_width; img_x++)
                        {
                            const uint16_t value = image[img_y*frame_width + img_x];
                            msg.data.push_back(value >> 8);
                        }
                    }

                    // time
                    msg.header.stamp = reset_time_ +
                            ros::Duration().fromNSec(caerFrameEventGetTimestamp64(event, frame) * 1000);

                    image_pub_.publish(msg);

                    // publish image exposure
                    const int32_t exposure_time_microseconds = caerFrameEventGetExposureLength(event);
                    std_msgs::Int32 exposure_msg;
                    exposure_msg.data = exposure_time_microseconds;
                    if(exposure_pub_.getNumSubscribers() > 0)
                    {
                      exposure_pub_.publish(exposure_msg);
                    }

                    // auto-exposure algorithm
                    if(current_config_.autoexposure_enabled)
                    {
                        // using the requested exposure instead of the actual, measured one gives more stable results
                        uint32_t current_exposure;
                        caerDeviceConfigGet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, &current_exposure);
                        const int new_exposure = computeNewExposure(msg.data, current_exposure);

                        caerDeviceConfigSet(davis_handle_, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, new_exposure);
                    }
                }
            }

            caerEventPacketContainerFree(packetContainer);

            ros::spinOnce();
        }
        catch(boost::thread_interrupted&)
        {
            return;
        }
    }
    
    caerDeviceDataStop(davis_handle_);

}

int DavisRosDriver::computeNewExposure(const std::vector<uint8_t>& img_data, const uint32_t current_exposure) const
{
    const float desired_intensity = static_cast<float>(current_config_.autoexposure_desired_intensity);
    static constexpr int min_exposure = 10;
    static constexpr int max_exposure = 25000;
    static constexpr float proportion_to_cut = 0.25f;

    const float current_intensity = trim_mean(img_data, proportion_to_cut);

    const float err = desired_intensity - current_intensity;
    const float delta_exposure = static_cast<float>(current_exposure) * static_cast<float>(current_config_.autoexposure_gain) / 1000.f * err;

    const int new_exposure = static_cast<int> (static_cast<float>(current_exposure) + delta_exposure + 0.5f);

    return clip(new_exposure, min_exposure, max_exposure);
}

} // namespace

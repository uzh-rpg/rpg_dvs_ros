// This file is part of DVS-ROS - the RPG DVS ROS Package

#include "dvs_ros_driver/driver.h"

namespace dvs_ros_driver {

DvsRosDriver::DvsRosDriver(ros::NodeHandle & nh, ros::NodeHandle nh_private) :
    nh_(nh), parameter_update_required_(false)
{
  // load parameters
  nh_private.param<std::string>("serial_number", device_id_, "");
  nh_private.param<bool>("master", master_, true);
  double reset_timestamps_delay;
  nh_private.param<double>("reset_timestamps_delay", reset_timestamps_delay, -1.0);

  // start driver
  bool device_is_running = false;
  while (!device_is_running)
  {
    const char* serial_number_restrict = (device_id_ == "") ? NULL : device_id_.c_str();
    dvs128_handle = caerDeviceOpen(1, CAER_DEVICE_DVS128, 0, 0, serial_number_restrict);

    //dvs_running = driver_->isDeviceRunning();
    device_is_running = !(dvs128_handle == NULL);

    if (!device_is_running)
    {
      ROS_WARN("Could not find DVS. Will retry every second.");
      ros::Duration(1.0).sleep();
      ros::spinOnce();
    }
    else
    {
      // configure as master or slave
      caerDeviceConfigSet(dvs128_handle, DVS128_CONFIG_DVS, DVS128_CONFIG_DVS_TS_MASTER, master_);
    }

    if (!ros::ok())
    {
      return;
    }
  }

  dvs128_info_ = caerDVS128InfoGet(dvs128_handle);
  device_id_ = "DVS128-V1-" + std::string(dvs128_info_.deviceString).substr(15, 4);

  ROS_INFO("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Logic: %d.\n", dvs128_info_.deviceString,
           dvs128_info_.deviceID, dvs128_info_.deviceIsMaster, dvs128_info_.dvsSizeX, dvs128_info_.dvsSizeY,
           dvs128_info_.firmwareVersion);

  current_config_.streaming_rate = 30;
  delta_ = boost::posix_time::microseconds(long(1e6 / current_config_.streaming_rate));

  // set namespace
  std::string ns = ros::this_node::getNamespace();
  if (ns == "/")
    ns = "/dvs";
  event_array_pub_ = nh_.advertise<dvs_msgs::EventArray>(ns + "/events", 1);
  camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(ns + "/camera_info", 1);

  // camera info handling
  ros::NodeHandle nh_ns(ns);
  camera_info_manager_ = new camera_info_manager::CameraInfoManager(nh_ns, device_id_);

  // reset timestamps is publisher as master, subscriber as slave
  if (master_)
  {
    reset_pub_ = nh_.advertise<std_msgs::Time>((ns + "/reset_timestamps").c_str(), 1);
  }
  else
  {
    reset_sub_ = nh_.subscribe((ns + "/reset_timestamps").c_str(), 1, &DvsRosDriver::resetTimestampsCallback, this);
  }

  // initialize timestamps
  resetTimestamps();

  // spawn threads
  running_ = true;
  parameter_thread_ = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&DvsRosDriver::changeDvsParameters, this)));
  readout_thread_ = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&DvsRosDriver::readout, this)));

  // Dynamic reconfigure
  dynamic_reconfigure_callback_ = boost::bind(&DvsRosDriver::callback, this, _1, _2);
  server_.reset(new dynamic_reconfigure::Server<dvs_ros_driver::DVS_ROS_DriverConfig>(nh_private));
  server_->setCallback(dynamic_reconfigure_callback_);

  // start timer to reset timestamps for synchronization
  if (reset_timestamps_delay > 0.0 && master_)
  {
    timestamp_reset_timer_ = nh_.createTimer(ros::Duration(reset_timestamps_delay), &DvsRosDriver::resetTimerCallback, this);
    ROS_INFO("Started timer to reset timestamps on master DVS for synchronization (delay=%3.2fs).", reset_timestamps_delay);
  }
}

DvsRosDriver::~DvsRosDriver()
{
  if (running_)
  {
    ROS_INFO("shutting down threads");
    running_ = false;
    parameter_thread_->join();
    readout_thread_->join();
    ROS_INFO("threads stopped");

    caerDeviceClose(&dvs128_handle);
  }
}

void DvsRosDriver::resetTimestamps()
{
  if (caerDeviceConfigSet(dvs128_handle, DVS128_CONFIG_DVS, DVS128_CONFIG_DVS_TIMESTAMP_RESET, 1))
  {
    ROS_INFO("Reset timestamps on %s", device_id_.c_str());
    reset_time_ = ros::Time::now();

    // if master, publish reset time to slaves
    if (master_)
    {
      std_msgs::Time reset_msg;
      reset_msg.data = reset_time_;
      reset_pub_.publish(reset_msg);
    }
  }
  else
  {
    ROS_ERROR("Failed to reset timestamps on %s", device_id_.c_str());
  }
}

void DvsRosDriver::resetTimestampsCallback(const std_msgs::Time::ConstPtr &msg)
{
  // if slave, only adjust offset time
  if (!dvs128_info_.deviceIsMaster)
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

void DvsRosDriver::resetTimerCallback(const ros::TimerEvent& te)
{
  timestamp_reset_timer_.stop();
  resetTimestamps();
}

void DvsRosDriver::changeDvsParameters()
{
  while(running_)
  {
    try
    {
      if (parameter_update_required_)
      {
        parameter_update_required_ = false;
        caerDeviceConfigSet(dvs128_handle, DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_CAS, current_config_.cas);
        caerDeviceConfigSet(dvs128_handle, DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_INJGND, current_config_.injGnd);
        caerDeviceConfigSet(dvs128_handle, DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_REQPD, current_config_.reqPd);
        caerDeviceConfigSet(dvs128_handle, DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_PUX, current_config_.puX);
        caerDeviceConfigSet(dvs128_handle, DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_DIFFOFF, current_config_.diffOff);
        caerDeviceConfigSet(dvs128_handle, DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_REQ, current_config_.req);
        caerDeviceConfigSet(dvs128_handle, DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_REFR, current_config_.refr);
        caerDeviceConfigSet(dvs128_handle, DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_PUY, current_config_.puY);
        caerDeviceConfigSet(dvs128_handle, DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_DIFFON, current_config_.diffOn);
        caerDeviceConfigSet(dvs128_handle, DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_DIFF, current_config_.diff);
        caerDeviceConfigSet(dvs128_handle, DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_FOLL, current_config_.foll);
        caerDeviceConfigSet(dvs128_handle, DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_PR, current_config_.Pr);
      }

      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
    catch(boost::thread_interrupted&)
    {
      return;
    }
  }
}

void DvsRosDriver::callback(dvs_ros_driver::DVS_ROS_DriverConfig &config, uint32_t level)
{
  // did any DVS bias setting change?
   if (current_config_.cas != config.cas || current_config_.injGnd != config.injGnd ||
       current_config_.reqPd != config.reqPd || current_config_.puX != config.puX ||
       current_config_.diffOff != config.diffOff || current_config_.req != config.req ||
       current_config_.refr != config.refr || current_config_.puY != config.puY ||
       current_config_.diffOn != config.diffOn || current_config_.diff != config.diff ||
       current_config_.foll != config.foll || current_config_.Pr != config.Pr) {

     current_config_.cas = config.cas;
     current_config_.injGnd = config.injGnd;
     current_config_.reqPd = config.reqPd;
     current_config_.puX = config.puX;
     current_config_.diffOff = config.diffOff;
     current_config_.req = config.req;
     current_config_.refr = config.refr;
     current_config_.puY = config.puY;
     current_config_.diffOn = config.diffOn;
     current_config_.diff = config.diff;
     current_config_.foll = config.foll;
     current_config_.Pr = config.Pr;

     parameter_update_required_ = true;
   }

   // change streaming rate, if necessary
   if (current_config_.streaming_rate != config.streaming_rate) {
     current_config_.streaming_rate = config.streaming_rate;
     if (current_config_.streaming_rate > 0)
     {
       delta_ = boost::posix_time::microseconds(long(1e6/current_config_.streaming_rate));
     }
   }
}

void DvsRosDriver::readout()
{
  caerDeviceDataStart(dvs128_handle, NULL, NULL, NULL, NULL, NULL);
  caerDeviceConfigSet(dvs128_handle, CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

  boost::posix_time::ptime next_send_time = boost::posix_time::microsec_clock::local_time();

  dvs_msgs::EventArrayPtr event_array_msg(new dvs_msgs::EventArray());
  event_array_msg->height = dvs128_info_.dvsSizeY;
  event_array_msg->width = dvs128_info_.dvsSizeX;

  while (running_)
  {
    try
    {
      caerEventPacketContainer packetContainer = caerDeviceDataGet(dvs128_handle);
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
          caerPolarityEventPacket polarity = (caerPolarityEventPacket) packetHeader;

          const int numEvents = caerEventPacketHeaderGetEventNumber(packetHeader);

          for (int j = 0; j < numEvents; j++)
          {
            // Get full timestamp and addresses of first event.
            caerPolarityEvent event = caerPolarityEventPacketGetEvent(polarity, j);

            dvs_msgs::Event e;
            e.x = caerPolarityEventGetX(event);
            e.y = caerPolarityEventGetY(event);
            e.ts = reset_time_ +
              ros::Duration().fromNSec(caerPolarityEventGetTimestamp64(event, polarity) * 1000);
            e.polarity = caerPolarityEventGetPolarity(event);

            event_array_msg->events.push_back(e);
          }

          // throttle event messages
          if (boost::posix_time::microsec_clock::local_time() > next_send_time || current_config_.streaming_rate == 0)
          {
            event_array_pub_.publish(event_array_msg);
            event_array_msg->events.clear();
            if (current_config_.streaming_rate > 0)
            {
              next_send_time += delta_;
            }
          }

          if (camera_info_manager_->isCalibrated())
          {
            sensor_msgs::CameraInfoPtr camera_info_msg(new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
            camera_info_pub_.publish(camera_info_msg);
          }
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

  caerDeviceDataStop(dvs128_handle);
}

} // namespace

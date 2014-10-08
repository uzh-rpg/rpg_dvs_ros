#include "dvs_ros_driver/driver.h"

namespace dvs_ros_driver {

DvsRosDriver::DvsRosDriver()
{
  parameter_update_required = false;

  // load parameters
  ros::NodeHandle nh_private("~");
  std::string dvs_serial_number;
  nh_private.param<std::string>("serial_number", dvs_serial_number, "");
  bool master;
  nh_private.param<bool>("master", master, true);
  double reset_timestamps_delay;
  nh_private.param<double>("reset_timestamps_delay", reset_timestamps_delay, -1.0);

  // start driver
  driver = new dvs::DVS_Driver(dvs_serial_number, master);

  // camera info handling
  cameraInfoManager = new camera_info_manager::CameraInfoManager(nh, driver->get_camera_id());

  current_config.streaming_rate = 30;
  delta = boost::posix_time::microseconds(1e6/current_config.streaming_rate);

  // set namespace
  std::string ns = ros::this_node::getNamespace();
  if (ns == "/")
    ns = "/dvs";
  event_array_pub = nh.advertise<dvs_msgs::EventArray>(ns + "/events", 1);
  camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>(ns + "/camera_info", 1);

  // create threads
  parameter_thread = new boost::thread(boost::bind(&DvsRosDriver::change_dvs_parameters, this));
  readout_thread = new boost::thread(boost::bind(&DvsRosDriver::readout, this));

  reset_sub = nh.subscribe((ns + "/reset_timestamps").c_str(), 1, &DvsRosDriver::reset_timestamps, this);

  // Dynamic reconfigure
  f = boost::bind(&DvsRosDriver::callback, this, _1, _2);
  server.setCallback(f);

  // reset timestamps for synchronization
  if (reset_timestamps_delay > 0.0) {
    ros::Duration(reset_timestamps_delay).sleep();
    driver->resetTimestamps();
    ROS_INFO("Reset timestamps on master DVS for synchronization (delay=%3.2fs).", reset_timestamps_delay);
  }
}

DvsRosDriver::~DvsRosDriver()
{
  parameter_thread->interrupt();
  readout_thread->interrupt();
}

void DvsRosDriver::reset_timestamps(std_msgs::Empty msg) {
  ROS_INFO("Reset timestamps on %s", driver->get_camera_id().c_str());
  driver->resetTimestamps();
}

void DvsRosDriver::change_dvs_parameters() {
  while(true) {
    try {
      if (parameter_update_required) {
        parameter_update_required = false;
        driver->change_parameters(current_config.cas, current_config.injGnd, current_config.reqPd, current_config.puX,
                                  current_config.diffOff, current_config.req, current_config.refr, current_config.puY,
                                  current_config.diffOn, current_config.diff, current_config.foll, current_config.Pr);
      }

      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    } 
    catch(boost::thread_interrupted&) {
      return;
    }
  }
}

void DvsRosDriver::callback(dvs_ros_driver::DVS_ROS_DriverConfig &config, uint32_t level) {
  // did any DVS bias setting change?
   if (current_config.cas != config.cas || current_config.injGnd != config.injGnd ||
       current_config.reqPd != config.reqPd || current_config.puX != config.puX ||
       current_config.diffOff != config.diffOff || current_config.req != config.req ||
       current_config.refr != config.refr || current_config.puY != config.puY ||
       current_config.diffOn != config.diffOn || current_config.diff != config.diff ||
       current_config.foll != config.foll || current_config.Pr != config.Pr) {

     current_config.cas = config.cas;
     current_config.injGnd = config.injGnd;
     current_config.reqPd = config.reqPd;
     current_config.puX = config.puX;
     current_config.diffOff = config.diffOff;
     current_config.req = config.req;
     current_config.refr = config.refr;
     current_config.puY = config.puY;
     current_config.diffOn = config.diffOn;
     current_config.diff = config.diff;
     current_config.foll = config.foll;
     current_config.Pr = config.Pr;

     parameter_update_required = true;
   }

   // change streaming rate, if necessary
   if (current_config.streaming_rate != config.streaming_rate) {
     current_config.streaming_rate = config.streaming_rate;
     delta = boost::posix_time::microseconds(1e6/current_config.streaming_rate);
   }
}

void DvsRosDriver::readout() {
  std::vector<dvs::Event> events;
  dvs_msgs::EventArray msg;

  boost::posix_time::ptime next_send_time = boost::posix_time::microsec_clock::local_time();

  while(true) {
    try {
      events = driver->get_events();

      for (int i=0; i<events.size(); ++i) {
        dvs_msgs::Event e;
        e.x = events[i].x;
        e.y = events[i].y;
        e.time = events[i].timestamp;
        e.polarity = events[i].polarity;

        msg.events.push_back(e);
      }

      if (boost::posix_time::microsec_clock::local_time() > next_send_time)
      {
        if (cameraInfoManager->isCalibrated()) {
          camera_info_pub.publish(cameraInfoManager->getCameraInfo());
        }
        event_array_pub.publish(msg);
        ros::spinOnce();
        events.clear();
        msg.events.clear();

        next_send_time += delta;
      }

      boost::this_thread::sleep(delta/10.0);
    }
    catch(boost::thread_interrupted&) {
      return;
    }
  }
}

} // namespace

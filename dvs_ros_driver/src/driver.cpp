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

#include "dvs_ros_driver/driver.h"

namespace dvs_ros_driver {

DvsRosDriver::DvsRosDriver(ros::NodeHandle & nh, ros::NodeHandle nh_private) :
    nh_(nh)
{
  parameter_update_required = false;

  // load parameters
  std::string dvs_serial_number;
  nh_private.param<std::string>("serial_number", dvs_serial_number, "");
  bool master;
  nh_private.param<bool>("master", master, true);
  double reset_timestamps_delay;
  nh_private.param<double>("reset_timestamps_delay", reset_timestamps_delay, -1.0);

  // start driver
  driver = new dvs::DVS_Driver(dvs_serial_number, master);

  // camera info handling
  cameraInfoManager = new camera_info_manager::CameraInfoManager(nh_, driver->get_camera_id());

  current_config.streaming_rate = 30;
  delta = boost::posix_time::microseconds(1e6/current_config.streaming_rate);

  // set namespace
  std::string ns = ros::this_node::getNamespace();
  if (ns == "/")
    ns = "/dvs";
  event_array_pub = nh_.advertise<dvs_msgs::EventArray>(ns + "/events", 1);
  camera_info_pub = nh_.advertise<sensor_msgs::CameraInfo>(ns + "/camera_info", 1);

  // spawn threads
  running_ = true;
  parameter_thread = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&DvsRosDriver::change_dvs_parameters, this)));
  readout_thread = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&DvsRosDriver::readout, this)));

  reset_sub = nh_.subscribe((ns + "/reset_timestamps").c_str(), 1, &DvsRosDriver::reset_timestamps, this);

  // Dynamic reconfigure
  f = boost::bind(&DvsRosDriver::callback, this, _1, _2);
  server.reset(new dynamic_reconfigure::Server<dvs_ros_driver::DVS_ROS_DriverConfig>(nh_private));
  server->setCallback(f);

  // start timer to reset timestamps for synchronization
  if (reset_timestamps_delay > 0.0)
  {
    timestamp_reset_timer_ = nh_.createTimer(ros::Duration(reset_timestamps_delay), &DvsRosDriver::resetTimerCallback, this);
    ROS_INFO("Started timer to reset timestamps on master DVS for synchronization (delay=%3.2fs).", reset_timestamps_delay);
  }
}

DvsRosDriver::~DvsRosDriver()
{
  ROS_INFO("Destructor call");
  if (running_)
  {
    ROS_INFO("shutting down threads");
    running_ = false;
    parameter_thread->join();
    readout_thread->join();
    ROS_INFO("threads stopped");
  }
}

void DvsRosDriver::reset_timestamps(std_msgs::Empty msg)
{
  ROS_INFO("Reset timestamps on %s", driver->get_camera_id().c_str());
  driver->resetTimestamps();
}

void DvsRosDriver::resetTimerCallback(const ros::TimerEvent& te)
{
  ROS_INFO("Reset timestamps on %s", driver->get_camera_id().c_str());
  driver->resetTimestamps();
  timestamp_reset_timer_.stop();
}

void DvsRosDriver::change_dvs_parameters()
{
  while(running_)
  {
    try
    {
      if (parameter_update_required)
      {
        parameter_update_required = false;
        driver->change_parameters(current_config.cas, current_config.injGnd, current_config.reqPd, current_config.puX,
                                  current_config.diffOff, current_config.req, current_config.refr, current_config.puY,
                                  current_config.diffOn, current_config.diff, current_config.foll, current_config.Pr);
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

  boost::posix_time::ptime next_send_time = boost::posix_time::microsec_clock::local_time();

  while(running_)
  {
    try
    {
      events = driver->get_events();
      dvs_msgs::EventArrayPtr msg(new dvs_msgs::EventArray());

      for (int i=0; i<events.size(); ++i)
      {
        dvs_msgs::Event e;
        e.x = events[i].x;
        e.y = events[i].y;
        e.time = events[i].timestamp;
        e.polarity = events[i].polarity;

        msg->events.push_back(e);
      }

      if (cameraInfoManager->isCalibrated())
      {
        camera_info_pub.publish(cameraInfoManager->getCameraInfo());
      }
      event_array_pub.publish(msg);

//      ROS_INFO("Sending events %d", msg->events.size());

      ros::spinOnce();
      events.clear();

      next_send_time += delta;

      while (boost::posix_time::microsec_clock::local_time() < next_send_time)
      {
        boost::this_thread::sleep(delta/10.0);
      }
    }
    catch(boost::thread_interrupted&)
    {
      return;
    }
  }
}

} // namespace

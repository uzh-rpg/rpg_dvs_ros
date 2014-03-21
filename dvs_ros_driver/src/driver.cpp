#include "ros/ros.h"

// boost
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

// DVS driver
#include <dvs_driver/dvs_driver.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dvs_ros_driver/DVS_ROS_DriverConfig.h>

ros::Rate* loop_rate;
dvs::DVS_Driver* driver;

dvs_ros_driver::DVS_ROS_DriverConfig current_config;

bool parameter_update_required = false;

void change_dvs_parameters() {
  while(true) {
    try {
      if (parameter_update_required) {
        parameter_update_required = false;
        driver->change_parameters(current_config.cas, current_config.injGnd, current_config.reqPd, current_config.puX,
                                  current_config.diffOff, current_config.req, current_config.refr, current_config.puY,
                                  current_config.diffOn, current_config.diff, current_config.foll, current_config.pr);
      }

      boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    } 
    catch(boost::thread_interrupted&) {
      return;
    }
  }
}

void callback(dvs_ros_driver::DVS_ROS_DriverConfig &config, uint32_t level) {
  // did any DVS bias setting change?
   if (current_config.cas != config.cas || current_config.injGnd != config.injGnd ||
       current_config.reqPd != config.reqPd || current_config.puX != config.puX ||
       current_config.diffOff != config.diffOff || current_config.req != config.req ||
       current_config.refr != config.refr || current_config.puY != config.puY ||
       current_config.diffOn != config.diffOn || current_config.diff != config.diff ||
       current_config.foll != config.foll || current_config.pr != config.pr) {

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
     current_config.pr = config.pr;

     parameter_update_required = true;
   }

   // did streaming rate change?
   if (current_config.streaming_rate != config.streaming_rate) {
     current_config.streaming_rate = config.streaming_rate;
     loop_rate = new ros::Rate(current_config.streaming_rate );
   }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "dvs_ros_driver");

  ros::NodeHandle nh;

  current_config.streaming_rate = 30;
  loop_rate = new ros::Rate(current_config.streaming_rate);

  ros::Publisher event_array_pub = nh.advertise<dvs_msgs::EventArray>("dvs_events", 1);

  // Dynamic reconfigure
  dynamic_reconfigure::Server<dvs_ros_driver::DVS_ROS_DriverConfig> server;
  dynamic_reconfigure::Server<dvs_ros_driver::DVS_ROS_DriverConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // Load driver
  driver = new dvs::DVS_Driver();
  std::vector<dvs::Event> events;

  // start parameter updater
  boost::thread parameter_thread(&change_dvs_parameters);

  while (ros::ok()) {
    events.clear();
    dvs_msgs::EventArray msg;
    
    events = driver->get_events();

    for (int i=0; i<events.size(); ++i) {
      dvs_msgs::Event e;
      e.x = events[i].x;
      e.y = events[i].y;
      e.time = events[i].timestamp;
      e.polarity = events[i].polarity;

      msg.events.push_back(e);
    }

    event_array_pub.publish(msg);

    ros::spinOnce();

    loop_rate->sleep();
  }

  // end thread
  parameter_thread.interrupt();
  parameter_thread.join();

  return 0;
}

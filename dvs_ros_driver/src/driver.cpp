#include "ros/ros.h"

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

int streaming_rate = 30;

bool first_callback = true;
dvs_ros_driver::DVS_ROS_DriverConfig last_config;

void callback(dvs_ros_driver::DVS_ROS_DriverConfig &config, uint32_t level) {
  if (first_callback) {
    first_callback = false;
  }
  else {
    if (last_config.diffOff != config.diffOff) {
      ROS_INFO("Reconfigure Request for diffOff: %d (old: %d)", config.diffOff, last_config.diffOff);
      driver->change_parameter("diffOff", config.diffOff);
    }
    if (last_config.diffOn != config.diffOn) {
      ROS_INFO("Reconfigure Request for diffOn: %d (old: %d)", config.diffOn, last_config.diffOn);
      driver->change_parameter("diffOn", config.diffOn);
    }
  }

  // remember parameters
  last_config = config;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "dvs_ros_driver");

  ros::NodeHandle nh;

  loop_rate = new ros::Rate(streaming_rate);

  ros::Publisher event_array_pub = nh.advertise<dvs_msgs::EventArray>("dvs_events", 1);

  // Dynamic reconfigure
  dynamic_reconfigure::Server<dvs_ros_driver::DVS_ROS_DriverConfig> server;
  dynamic_reconfigure::Server<dvs_ros_driver::DVS_ROS_DriverConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // Load driver
  driver = new dvs::DVS_Driver();
  std::vector<dvs::Event> events;

  while (ros::ok()) {
    events.clear();
    dvs_msgs::EventArray msg;
    
    events = driver->get_events();

    ROS_WARN("New events! %d", events.size());

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

  return 0;
}

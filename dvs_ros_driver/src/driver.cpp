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

void callback(dvs_ros_driver::DVS_ROS_DriverConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %d %d", config.diffOff, config.diffOn, config.streaming_rate);
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

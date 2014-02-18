#include "ros/ros.h"
#include <fstream>
#include <ctime>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

std::ofstream myfile;

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d_%H-%M-%S", &tstruct);

    return buf;
}

void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg) {
  for (int i=0; i<msg->events.size(); ++i) {
    myfile << (int) msg->events[i].x << " ";
    myfile << (int) msg->events[i].y << " ";
    myfile << (int) msg->events[i].polarity << " ";
    myfile << msg->events[i].time << std::endl;
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dvs_file_writer");

  ros::NodeHandle nh;

  myfile.open (std::string("events-" + currentDateTime() + ".txt").c_str());

  ros::Subscriber sub = nh.subscribe("dvs_events", 1000, eventsCallback);

  ros::spin();

  myfile.close();

  return 0;
}

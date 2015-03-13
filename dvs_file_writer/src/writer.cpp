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

#include <ros/ros.h>
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

  std::string file_name = std::string("events-" + currentDateTime() + ".txt");

  myfile.open(file_name.c_str());

  ROS_INFO("Writing events to %s", file_name.c_str());
  ros::Subscriber sub = nh.subscribe("dvs/events", 1000, eventsCallback);

  ros::spin();

  myfile.close();

  return 0;
}

#include <ros/ros.h>
#include "dvs_ros_driver/driver.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dvs_ros_driver");

  dvs_ros_driver::DvsRosDriver* driver = new dvs_ros_driver::DvsRosDriver();

  ros::spin();
  return 0;
}

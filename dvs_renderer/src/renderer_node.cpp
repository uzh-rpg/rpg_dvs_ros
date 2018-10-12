// This file is part of DVS-ROS - the RPG DVS ROS Package

#include "dvs_renderer/renderer.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dvs_renderer");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  dvs_renderer::Renderer renderer(nh, nh_private);

  ros::spin();

  return 0;
}

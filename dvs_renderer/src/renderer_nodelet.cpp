// This file is part of DVS-ROS - the RPG DVS ROS Package

#include <pluginlib/class_list_macros.h>

#include "dvs_renderer/renderer_nodelet.h"

namespace dvs_renderer
{

void DvsRendererNodelet::onInit()
{
  renderer = new dvs_renderer::Renderer(getNodeHandle(), getPrivateNodeHandle());
  NODELET_INFO_STREAM("Initialized " <<  getName() << " nodelet.");
}

#ifndef PLUGINLIB_EXPORT_CLASS
PLUGINLIB_DECLARE_CLASS(dvs_renderer, DvsRendererNodelet, dvs_renderer::DvsRendererNodelet, nodelet::Nodelet);
#else
PLUGINLIB_EXPORT_CLASS(dvs_renderer::DvsRendererNodelet, nodelet::Nodelet);
#endif

}

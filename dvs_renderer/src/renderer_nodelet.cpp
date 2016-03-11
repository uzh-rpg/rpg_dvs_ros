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

#include <pluginlib/class_list_macros.h>

#include "dvs_renderer/renderer_nodelet.h"

namespace dvs_renderer
{

void DvsRendererNodelet::onInit()
{
  renderer = new dvs_renderer::Renderer(getNodeHandle(), getPrivateNodeHandle());
  NODELET_INFO_STREAM("Initialized " <<  getName() << " nodelet.");
}

PLUGINLIB_DECLARE_CLASS(dvs_renderer, DvsRendererNodelet, dvs_renderer::DvsRendererNodelet, nodelet::Nodelet);

}

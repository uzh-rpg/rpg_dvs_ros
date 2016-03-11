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

#ifndef DVS_RENDERER_NODELET_H_
#define DVS_RENDERER_NODELET_H_

#include <nodelet/nodelet.h>

#include "dvs_renderer/renderer.h"

namespace dvs_renderer {

class DvsRendererNodelet : public nodelet::Nodelet {
public:
  virtual void onInit();

private:
  dvs_renderer::Renderer* renderer;
};

}

#endif // DVS_RENDERER_NODELET_H_

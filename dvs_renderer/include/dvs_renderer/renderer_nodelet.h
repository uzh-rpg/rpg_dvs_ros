// This file is part of DVS-ROS - the RPG DVS ROS Package

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

// This file is part of DVS-ROS - the RPG DVS ROS Package

#ifndef BOARD_DETECTION_H
#define BOARD_DETECTION_H

#include <list>

#include <opencv2/calib3d/calib3d.hpp>

#include "dvs_calibration/circlesgrid.hpp"

namespace dvs_calibration {

struct PointWithWeight {
  cv::Point point;
  double weight;
};

class BoardDetection
{
public:
  static std::vector<cv::Point2f> findPattern(std::list<PointWithWeight> points, int dots_w, int dots_h, int minimum_points);
};

} // namespace

#endif // BOARD_DETECTION_H

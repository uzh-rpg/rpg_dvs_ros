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

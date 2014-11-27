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

#ifndef TRANSITION_MAP_H
#define TRANSITION_MAP_H

#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include <dvs_msgs/EventArray.h>

#include "dvs_calibration/board_detection.h"
#include "dvs_calibration/calibration_parameters.h"

namespace dvs_calibration {

class TransitionMap
{
public:
  TransitionMap(const CalibrationParameters params = CalibrationParameters());
  int max();
  void update(const dvs_msgs::EventArray::ConstPtr& msg);
  bool has_pattern() {
    return _has_pattern;
  }
  void find_pattern();

  std::vector<cv::Point2f> pattern;

  void reset_maps();

  cv::Mat get_visualization_image();

  ros::Time get_last_reset_time() {
    return last_reset_time;
  }

private:
  static const int sensor_width = 128;
  static const int sensor_height = 128;

  int last_off_map[sensor_width][sensor_height];
  int last_on_map[sensor_width][sensor_height];
  int transition_sum_map[sensor_width][sensor_height];

  CalibrationParameters params;

  bool _has_pattern;

  ros::Time last_reset_time;
};

} // namespace

#endif // TRANSITION_MAP_H

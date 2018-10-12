// This file is part of DVS-ROS - the RPG DVS ROS Package

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
  TransitionMap(const CalibrationParameters params_ = CalibrationParameters());
  int max();
  void update(const dvs_msgs::EventArray::ConstPtr& msg);
  bool has_pattern() {
    return has_pattern_;
  }
  void find_pattern();

  std::vector<cv::Point2f> pattern;

  void reset_maps();

  cv::Mat get_visualization_image();

  ros::Time get_last_reset_time() {
    return last_reset_time_;
  }

private:
  int sensor_width_;
  int sensor_height_;

  void init_transition_maps(const int width, const int height);
  std::vector<std::vector<ros::Time>> last_off_map_, last_on_map_;
  std::vector<std::vector<int>> transition_sum_map_;

  CalibrationParameters params_;

  bool has_pattern_;

  ros::Time last_reset_time_;
};

} // namespace

#endif // TRANSITION_MAP_H

#ifndef TRANSITION_MAP_H
#define TRANSITION_MAP_H

#include "dvs_calibration/board_detection.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <dvs_msgs/EventArray.h>
#include <vector>

class TransitionMap
{
public:
  TransitionMap();
  int max();
  void update(const dvs_msgs::EventArray::ConstPtr& msg);
  bool has_pattern();

  std::vector<cv::Point2f> pattern;

  void reset_maps();

  cv::Mat get_visualization_image();

private:
  void find_pattern();

  static const int sensor_width = 128;
  static const int sensor_height = 128;
  static const int blinking_time_us = 1000;
  static const int blinking_time_tolerance = 500;
  static const int enough_transitions_threshold = 200;
  static const int minimum_transitions_threshold = 10;
  static const int minimum_led_mass = 50;
  static const int dots = 5;
  static const double dot_distance = 0.05;

  int last_off_map[sensor_width][sensor_height];
  int last_on_map[sensor_width][sensor_height];
  int transition_sum_map[sensor_width][sensor_height];

  bool checked_pattern_since_last_update;
  bool _has_pattern;
};

#endif // TRANSITION_MAP_H

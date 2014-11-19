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

// This file is part of DVS-ROS - the RPG DVS ROS Package

#ifndef CALIBRATION_PARAMETERS_H
#define CALIBRATION_PARAMETERS_H

namespace dvs_calibration {

struct CalibrationParameters {
  int blinking_time_us;
  int blinking_time_tolerance_us;
  int enough_transitions_threshold;
  int minimum_transitions_threshold;
  int minimum_led_mass;
  int dots_w, dots_h;
  double dot_distance;
  double pattern_search_timeout;
};

} // namespace

#endif // CALIBRATION_PARAMETERS_H

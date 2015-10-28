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

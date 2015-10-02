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

#include "dvs_calibration/transition_map.h"

namespace dvs_calibration {

TransitionMap::TransitionMap(const CalibrationParameters params) : params_(params)
{
  has_pattern_ = false;

  reset_maps();
  last_reset_time_ = ros::Time::now();
}

int TransitionMap::max()
{
  int max = 0;
  for (int i = 0; i < sensor_width; i++)
  {
    for (int j = 0; j < sensor_height; j++)
    {
      if (transition_sum_map_[i][j] > max)
        max = transition_sum_map_[i][j];
    }
  }
  return max;
}

void TransitionMap::update(const dvs_msgs::EventArray::ConstPtr& msg)
{
  for (int i = 0; i < msg->events.size(); ++i)
  {
    if (msg->events[i].polarity == true)
    {
      last_off_map_[msg->events[i].x][msg->events[i].y] = msg->events[i].time;
    }
    else
    {
      int delta_t = msg->events[i].time - last_off_map_[msg->events[i].x][msg->events[i].y];
      if (delta_t < params_.blinking_time_us + params_.blinking_time_tolerance && delta_t > params_.blinking_time_us - params_.blinking_time_tolerance)
        transition_sum_map_[msg->events[i].x][msg->events[i].y]++;
    }
  }

}

cv::Mat TransitionMap::get_visualization_image()
{
  cv::Mat image = cv::Mat(sensor_height, sensor_width, CV_8UC3);
  image = cv::Scalar(255, 255, 255);
  int max_value = max();
  for (int i = 0; i < sensor_width; i++)
  {
    for (int j = 0; j < sensor_height; j++)
    {
      int value = 255.0 - ((double)transition_sum_map_[i][j]) / ((double)max_value) * 255.0;
      image.at<cv::Vec3b>(j, i) = cv::Vec3b(value, value, value);
    }
  }

  if (has_pattern())
  {
    cv::drawChessboardCorners(image, cv::Size(params_.dots_w, params_.dots_h), cv::Mat(pattern), true);
  }

  return image;
}

void TransitionMap::find_pattern()
{
  std::list<PointWithWeight> points;

  for (int i = 0; i < sensor_width; i++)
  {
    for (int j = 0; j < sensor_height; j++)
    {
      if (transition_sum_map_[i][j] > params_.minimum_transitions_threshold)
      {
        PointWithWeight p;
        p.point = cv::Point(i, j);
        p.weight = (double) transition_sum_map_[i][j];
        points.push_back(p);
      }
    }
  }

  pattern = BoardDetection::findPattern(points, params_.dots_w, params_.dots_h, params_.minimum_led_mass);

  if (pattern.size() == params_.dots_w * params_.dots_h)
  {
    has_pattern_ = true;
  }
}

void TransitionMap::reset_maps()
{
  last_reset_time_ = ros::Time::now();
  for (int i = 0; i < sensor_width; i++)
  {
    for (int j = 0; j < sensor_height; j++)
    {
      last_on_map_[i][j] = 0;
      last_off_map_[i][j] = 0;
      transition_sum_map_[i][j] = 0;
    }
  }
  has_pattern_ = false;
}

} // namespace

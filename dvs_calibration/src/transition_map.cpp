#include "dvs_calibration/transition_map.h"

TransitionMap::TransitionMap()
{
  checked_pattern_since_last_update = false;
  _has_pattern = false;
}

int TransitionMap::max()
{
  int max = 0;
  for (int i = 0; i < sensor_width; i++)
  {
    for (int j = 0; j < sensor_height; j++)
    {
      if (transition_sum_map[i][j] > max)
        max = transition_sum_map[i][j];
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
      last_off_map[msg->events[i].x][msg->events[i].y] = msg->events[i].time;
    }
    else
    {
      int delta_t = msg->events[i].time - last_off_map[msg->events[i].x][msg->events[i].y];
      if (delta_t < blinking_time_us + blinking_time_tolerance && delta_t > blinking_time_us - blinking_time_tolerance)
        transition_sum_map[msg->events[i].x][msg->events[i].y]++;
    }
  }

  checked_pattern_since_last_update = false;
}

bool TransitionMap::has_pattern()
{
  if (!checked_pattern_since_last_update) {
    find_pattern();
  }
  return _has_pattern;
}

cv::Mat TransitionMap::get_visualization_image()
{
  cv::Mat image = cv::Mat(sensor_height, sensor_width, CV_8UC3);
  image = cv::Scalar(255, 255, 255);
  for (int i = 0; i < sensor_width; i++)
  {
    for (int j = 0; j < sensor_height; j++)
    {
      int value = 255.0 - ((double)transition_sum_map[i][j]) / ((double)enough_transitions_threshold) * 255.0;
      image.at<cv::Vec3b>(j, i) = cv::Vec3b(value, value, value);
    }
  }

  if (has_pattern()) {
    cv::drawChessboardCorners(image, cv::Size(dots, dots), cv::Mat(pattern), true);
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
      if (transition_sum_map[i][j] > minimum_transitions_threshold)
      {
        PointWithWeight p;
        p.point = cv::Point(i, j);
        p.weight = (double) transition_sum_map[i][j];
        points.push_back(p);
      }
    }
  }

  pattern = BoardDetection::findPattern(points);
  _has_pattern = (pattern.size() == dots * dots);
}

void TransitionMap::reset_maps()
{
  for (int i = 0; i < sensor_width; i++)
  {
    for (int j = 0; j < sensor_height; j++)
    {
      last_on_map[i][j] = 0;
      last_off_map[i][j] = 0;
      transition_sum_map[i][j] = 0;
    }
  }
}

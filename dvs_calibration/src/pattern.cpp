#include "dvs_calibration/pattern.h"

Pattern::Pattern(int pattern_size, int border_size, int dots, bool blinking) :
    pattern_size(pattern_size), border_size(border_size), dots_(dots), blinking(blinking)
{
  init_pattern_points();
}

cv::Mat Pattern::get_window_outline_pattern()
{
  cv::Mat pattern = cv::Mat(pattern_size, pattern_size, CV_8U);
  pattern = cv::Scalar(0);
  double p = 0.5 * border_size / 100.0 * pattern_size;
  cv::Point p1(p, p);
  cv::Point p2(pattern_size - p, pattern_size - p);
  cv::rectangle(pattern, p1, p2, cv::Scalar(255), 5);
  return pattern;
}

cv::Mat Pattern::get_focus_adjustment_pattern()
{
  cv::Mat pattern = cv::Mat(pattern_size, pattern_size, CV_8U);
  double size = pattern_size / 2.0;
  bool white = false;
  while (size > 1)
  {
    cv::Scalar color;
    if (white)
    {
      white = false;
      color = cv::Scalar(255);
    }
    else
    {
      white = true;
      color = cv::Scalar(0);
    }
    cv::rectangle(pattern, cv::Point(pattern_size / 2.0 - size, pattern_size / 2.0 - size),
                  cv::Point(pattern_size / 2.0 + size, pattern_size / 2.0 + size), color, -1);

    size /= 1.2;
  }

  return pattern;
}

cv::Mat Pattern::get_intrinsic_calibration_pattern(Eigen::Matrix3d orientation)
{
  cv::Mat pattern = cv::Mat(pattern_size, pattern_size, CV_8U);
  pattern = cv::Scalar(0);

  // transform all points
  std::vector<Eigen::Vector2d> image_points;
  for (int i = 0; i < pattern_points_.size(); i++)
  {
    Eigen::Vector3d p = orientation * pattern_points_[i];
    image_points.push_back(Eigen::Vector2d(p.x() / (p.z()+pattern_distance_), p.y() / (p.z()+pattern_distance_)));
  }

  // calculate drawing area
  double border = 0.5 * border_size/100.0*pattern_size;
  double drawing_field = pattern_size - 2 * dot_radius_ - 2 * border;

  // pattern is from -1 to 1
  // max_dist is maximum projection size
  double x = sqrt(2.0)/pattern_distance_;
  double max_dist =  x / sqrt(1 - x*x);
  double factor = 0.5 * drawing_field / max_dist;

  for (int i = 0; i < image_points.size(); i++)
  {
    cv::circle(
        pattern,
        cv::Point(pattern_size/2.0 + image_points[i].x() * factor,
              pattern_size/2.0 + image_points[i].y() * factor),
              dot_radius_, cv::Scalar(255), -1);
  }

  return pattern;
}

void Pattern::set_dots(int dots)
{
  dots_ = dots;
  init_pattern_points();
}

void Pattern::init_pattern_points()
{
  pattern_points_.clear();
  for (int i = 0; i < dots_; i++)
  {
    for (int j = 0; j < dots_; j++)
    {
      double x = (i - (dots_ - 1) / 2.0) / (0.5 * (dots_ - 1));
      double y = (j - (dots_ - 1) / 2.0) / (0.5 * (dots_ - 1));
      pattern_points_.push_back(Eigen::Vector3d(x, y, 0.0));
    }
  }
}

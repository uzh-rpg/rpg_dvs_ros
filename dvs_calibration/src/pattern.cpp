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
    image_points.push_back(Eigen::Vector2d(p.x() / p.z(), p.y() / p.z()));
  }

  // find minimum and maximum to scale appropriately
  Eigen::Vector2d min = image_points[0];
  Eigen::Vector2d max = image_points[0];
  for (int i = 1; i < image_points.size(); i++)
  {
    min.x() = std::min(min.x(), image_points[i].x());
    min.y() = std::min(min.y(), image_points[i].y());
    max.x() = std::max(max.x(), image_points[i].x());
    max.y() = std::max(max.y(), image_points[i].y());
  }

  double width = max.x() - min.x();
  double height = max.y() - min.y();

  double border = 0.5 * border_size/100.0*pattern_size;
  double factor = (pattern_size - 2 * dot_radius_ - 2 * border) / std::max(width, height);

  for (int i = 0; i < image_points.size(); i++)
  {
    cv::circle(
        pattern,
        cv::Point(border + dot_radius_ + (image_points[i].x() - min.x()) * factor,
              border + dot_radius_ + (image_points[i].y() - min.y()) * factor),
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
      pattern_points_.push_back(Eigen::Vector3d(x, y, 10.0));
    }
  }
}

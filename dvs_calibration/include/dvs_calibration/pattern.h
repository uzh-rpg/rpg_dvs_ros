#ifndef PATTERN_H
#define PATTERN_H

#include <opencv2/core/core.hpp>
#include <Eigen/Geometry>
#include <vector>

class Pattern
{
public:
  Pattern(int pattern_size = 1000, int border_size = 10, int dots = 4);
  int pattern_size;
  int border_size;

  cv::Mat get_window_outline_pattern();
  cv::Mat get_focus_adjustment_pattern();
  cv::Mat get_intrinsic_calibration_pattern(double roll, double pitch);

  void set_dots(int dots);

private:
  static const int dot_radius_ = 10;

  std::vector<Eigen::Vector3d> pattern_points_;
  int dots_;

  void init_pattern_points();
};

#endif // PATTERN_H

#ifndef BOARD_DETECTION_H
#define BOARD_DETECTION_H

#include "dvs_calibration/circlesgrid.hpp"
#include <opencv2/calib3d/calib3d.hpp>

#include <list>

struct PointWithWeight {
  cv::Point point;
  double weight;
};

class BoardDetection
{
public:
  static std::vector<cv::Point2f> findPattern(std::list<PointWithWeight> points, int dots = 5, int minimum_points = 50);
};

#endif // BOARD_DETECTION_H

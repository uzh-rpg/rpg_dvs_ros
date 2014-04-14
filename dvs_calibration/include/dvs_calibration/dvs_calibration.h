#ifndef DVS_CALIBRATION_H
#define DVS_CALIBRATION_H

#include "ros/ros.h"

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include "dvs_calibration/pattern.h"
#include "dvs_calibration/circlesgrid.hpp"
#include <dynamic_reconfigure/server.h>
#include <dvs_calibration/DvsCalibrationConfig.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//tmp
#include <opencv2/highgui/highgui.hpp>

class DvsCalibration
{
public:
  DvsCalibration();

private:
  // parameters
  static const int sensor_width = 128;
  static const int sensor_height = 128;
  static const int max_transition_time_us = 10000;
  static const int enough_transitions_threshold = 200;

  // mode
  enum {WINDOW_OUTLINE, FOCUS_ADJUSTMENT, INTRINSIC_CALIBRATION} mode;

  // event maps
  void reset_maps();
  int last_off_map[sensor_width][sensor_height];
  int transition_sum_map[sensor_width][sensor_height];

  // pattern
  int dots;
  Pattern pattern;
  std::vector<cv::Point2f> findPattern();
  void publishVisualizationImage(std::vector<cv::Point2f> pattern);
  void publishPatternImage(cv::Mat image);

  // calibration stuff
  void calibrate();
  void resetIntrinsicCalibration();
  std::vector< std::vector<cv::Point3f> > object_points;
  std::vector< std::vector<cv::Point2f> > image_points;

  // callbacks
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void dynamicReconfigureCallback(dvs_calibration::DvsCalibrationConfig &config, uint32_t level);

  // ROS interface
  ros::NodeHandle nh;
  ros::Subscriber eventSubscriber;
  image_transport::Publisher visualizationPublisher;
  image_transport::Publisher patternPublisher;

  dynamic_reconfigure::Server<dvs_calibration::DvsCalibrationConfig> srv;
  dynamic_reconfigure::Server<dvs_calibration::DvsCalibrationConfig>::CallbackType f;
};

#endif // DVS_CALIBRATION_H

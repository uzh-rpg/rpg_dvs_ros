#ifndef DVS_CALIBRATION_H
#define DVS_CALIBRATION_H

#include "ros/ros.h"

#include <Eigen/Geometry>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include "dvs_calibration/pattern.h"
#include "dvs_calibration/circlesgrid.hpp"
#include <dynamic_reconfigure/server.h>
#include <dvs_calibration/DvsCalibrationConfig.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <list>

class DvsCalibration
{
public:
  DvsCalibration();

private:
  // parameters
  static const int sensor_width = 128;
  static const int sensor_height = 128;
  static const int blinking_time_us = 1000;
  static const int blinking_time_tolerance = 500;
  static const int enough_transitions_threshold = 200;
  static const int minimum_transitions_threshold = 10;

  // mode
  enum {WINDOW_OUTLINE, FOCUS_ADJUSTMENT, INTRINSIC_CALIBRATION} mode;

  // event maps
  void reset_maps();
  int last_off_map[sensor_width][sensor_height];
  int last_on_map[sensor_width][sensor_height];
  int transition_sum_map[sensor_width][sensor_height];

  // pattern
  int dots;
  Pattern pattern;
  std::vector<cv::Point2f> findPattern();
  void publishVisualizationImage(std::vector<cv::Point2f> pattern);
  void publishPatternImage(cv::Mat image);
  std::vector<Eigen::Matrix3d> orientations;
  int orientation_id;

  // calibration stuff
  bool instrinsic_calibration_running_;
  void calibrate();
  void resetIntrinsicCalibration();
  std::vector< std::vector<cv::Point3f> > object_points;
  std::vector< std::vector<cv::Point2f> > image_points;

  // callbacks
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void dynamicReconfigureCallback(dvs_calibration::DvsCalibrationConfig &config, uint32_t level);
  bool startCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool resetCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  // ROS interface
  ros::NodeHandle nh;
  ros::Subscriber eventSubscriber;
  ros::Publisher detectionPublisher;
  ros::Publisher cameraInfoPublisher;
  ros::Publisher reprojectionErrorPublisher;
  image_transport::Publisher visualizationPublisher;
  image_transport::Publisher patternPublisher;
  ros::ServiceServer startCalibrationService;
  ros::ServiceServer resetService;

  dynamic_reconfigure::Server<dvs_calibration::DvsCalibrationConfig> srv;
  dynamic_reconfigure::Server<dvs_calibration::DvsCalibrationConfig>::CallbackType f;
};

#endif // DVS_CALIBRATION_H

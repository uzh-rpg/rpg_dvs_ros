#ifndef STEREO_CALIBRATION_H
#define STEREO_CALIBRATION_H

#include "ros/ros.h"

#include <Eigen/Geometry>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include "dvs_calibration/pattern.h"
#include "dvs_calibration/circlesgrid.hpp"
#include "dvs_calibration/board_detection.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/PoseStamped.h>

#include <boost/bind.hpp>

#include <list>

class StereoCalibration
{
public:
  StereoCalibration();

private:
  // parameters
  static const int sensor_width = 128;
  static const int sensor_height = 128;
  static const int blinking_time_us = 1000;
  static const int blinking_time_tolerance = 500;
  static const int enough_transitions_threshold = 200;
  static const int minimum_transitions_threshold = 10;
  static const int minimum_led_mass = 50;
  static const int dots = 5;
  static const double dot_distance = 0.05;

  static const int left_camera_id = 1;
  static const int right_camera_id = 2;

  static const double pattern_search_timeout = 2.0;
  ros::Time last_pattern_found;

  // event maps
  void reset_maps();
  int last_off_map[sensor_width][sensor_height];
  int last_on_map[sensor_width][sensor_height];
  int transition_sum_map[sensor_width][sensor_height];

  std::vector<cv::Point3f> world_pattern;

  // status
  bool calibration_running;
  bool gotCameraInfo;

  // pattern
  Pattern pattern;
  std::vector<cv::Point2f> findPattern();
  void publishVisualizationImage(std::vector<cv::Point2f> pattern);
  void publishPatternImage(cv::Mat image);
  int num_detections;

  // calibration stuff
  void calibrate();
  void resetIntrinsicCalibration();
  std::vector< std::vector<cv::Point3f> > object_points;
  std::vector< std::vector<cv::Point2f> > image_points;

  // callbacks
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  bool startCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool saveCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  bool resetCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  ros::Subscriber leftEventSub, rightEventSub;
  void eventsCallback2(const dvs_msgs::EventArray::ConstPtr& msg, int camera_id);

  // services
  sensor_msgs::CameraInfo cameraInfo;
  bool setCameraInfo();

  // ROS interface
  ros::NodeHandle nh;
  ros::Subscriber eventSubscriber;
  ros::Subscriber cameraInfoSubscriber;
  ros::Publisher detectionPublisher;
  ros::Publisher cameraInfoPublisher;
  ros::Publisher reprojectionErrorPublisher;
  ros::Publisher cameraPosePublisher;
  image_transport::Publisher visualizationPublisher;
  image_transport::Publisher patternPublisher;
  ros::ServiceServer startCalibrationService;
  ros::ServiceServer saveCalibrationService;
  ros::ServiceServer resetService;
  ros::ServiceClient setCameraInfoClient;

};

#endif // STEREO_CALIBRATION_H

// This file is part of DVS-ROS - the RPG DVS ROS Package

#ifndef STEREO_DVS_CALIBRATION_H
#define STEREO_DVS_CALIBRATION_H

#include <list>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "dvs_calibration/dvs_calibration.h"


namespace dvs_calibration {

class StereoDvsCalibration : public DvsCalibration
{
public:
  StereoDvsCalibration();
  virtual ~StereoDvsCalibration() {}

private:
  // callbacks
  void cameraInfoLeftCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void cameraInfoRightCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void resetCalibration();
  void startCalibration();
  void saveCalibration();

  void addPattern(int id);
  void updateVisualization(int id);

  // services
  sensor_msgs::CameraInfo camera_info_left_, camera_info_right_;
  bool setCameraInfo();

  void calibrate();
  void publishVisualizationImage(cv::Mat image, int id);

  // calibration
  std::vector< std::vector<cv::Point3f> > object_points_;
  std::vector< std::vector<cv::Point2f> > image_points_left_, image_points_right_;

  // ROS interface
  ros::Subscriber event_left_sub_, event_right_sub_;
  ros::Subscriber camera_info_left_sub_, camera_info_right_sub_;
  image_transport::Publisher visualization_left_pub_, visualization_right_pub_;
  ros::ServiceClient set_camera_info_left_client_, set_camera_info_right_client_;

  // buffer to wait for other camera
  void addStereoPattern(std::vector<cv::Point2f> left, std::vector<cv::Point2f> right);
  std::vector<cv::Point2f> image_point_buffer_;
  bool has_left_buffer_, has_right_buffer_;
  ros::Time buffer_time_;

  // for pose publishing
  bool got_camera_info_left_, got_camera_info_right_;
};

} // namespace

#endif // STEREO_DVS_CALIBRATION_H

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

#ifndef STEREO_DVS_CALIBRATION_H
#define STEREO_DVS_CALIBRATION_H

#include <list>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/version.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dvs_calibration/circlesgrid.hpp>

#include "dvs_calibration/dvs_calibration.h"


namespace dvs_calibration {

class CameraDvsCalibration : public DvsCalibration
{
public:
  CameraDvsCalibration();
  virtual ~CameraDvsCalibration() {}

private:
  // callbacks
  void standardCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void dvsCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

  void resetCalibration();
  void startCalibration();
  void saveCalibration();

  void addPattern(int id);
  void updateVisualization(int id);

  // services
  sensor_msgs::CameraInfo standard_camera_info_, dvs_camera_info_;
  sensor_msgs::CameraInfo new_standard_camera_info_, new_dvs_camera_info_;
  bool setCameraInfo();

  void calibrate();
  void publishVisualizationImage(cv::Mat image, int id);

  // calibration
  std::vector< std::vector<cv::Point3f> > object_points_;
  std::vector< std::vector<cv::Point2f> > image_points_camera_, image_points_dvs_;

  // ROS interface
  ros::Subscriber event_sub_, image_sub_;
  ros::Subscriber standard_camera_info_sub_, dvs_camera_info_sub_;
  image_transport::Publisher camera_visualization_pub_, dvs_visualization_pub_;
  ros::ServiceClient set_standard_camera_info_client_, set_dvs_camera_info_client_;

  // buffer to wait for other camera
  void add_stereo_pattern(std::vector<cv::Point2f> camera_image_points, std::vector<cv::Point2f> dvs_image_points);
  std::vector<cv::Point2f> camera_image_point_buffer_;
  ros::Time last_detection_camera_time_;

  // parameters
  int image_led_threshold_;
  double max_time_difference_;
  bool fix_intrinsics_;

  // for pose publishing
  bool got_standard_camera_info_, got_dvs_camera_info_;
};

} // namespace

#endif // STEREO_DVS_CALIBRATION_H

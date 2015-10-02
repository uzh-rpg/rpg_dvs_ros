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

#ifndef MONO_DVS_CALIBRATION_H
#define MONO_DVS_CALIBRATION_H

#include <list>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "dvs_calibration/dvs_calibration.h"


namespace dvs_calibration {

class MonoDvsCalibration : public DvsCalibration
{
public:
  MonoDvsCalibration();
  virtual ~MonoDvsCalibration() {}

private:
  void resetCalibration();
  void startCalibration();
  void saveCalibration();

  void addPattern(int id);
  void updateVisualization(int id);

  // services
  sensor_msgs::CameraInfo camera_info_external_;
  sensor_msgs::CameraInfo new_camera_info_;
  bool setCameraInfo();

  void calibrate();
  void publishVisualizationImage(cv::Mat image);

  // calibration
  std::vector< std::vector<cv::Point3f> > object_points_;
  std::vector< std::vector<cv::Point2f> > image_points_;

  // ROS interface
  ros::Subscriber event_sub_;
  ros::Publisher camera_pose_pub_;
  image_transport::Publisher visualization_pub_;
  image_transport::Publisher pattern_pub_;
  ros::ServiceClient set_camera_info_client_;
  ros::Subscriber camera_info_sub_;
  bool got_camera_info_;
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
};

} // namespace

#endif // MONO_DVS_CALIBRATION_H

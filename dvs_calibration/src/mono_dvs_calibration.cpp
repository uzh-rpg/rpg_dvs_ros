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

#include "dvs_calibration/mono_dvs_calibration.h"

namespace dvs_calibration {

MonoDvsCalibration::MonoDvsCalibration()
{
  calibration_running = false;

  setCameraInfoClient = nh.serviceClient<sensor_msgs::SetCameraInfo>("set_camera_info");

  // add transition map
  transition_maps.insert(std::pair<int, TransitionMap>(mono_camera_id, TransitionMap(params)));
  eventSubscriber = nh.subscribe<dvs_msgs::EventArray>("events", 1,
                                                       boost::bind(&MonoDvsCalibration::eventsCallback, this, _1, mono_camera_id));

  image_transport::ImageTransport it(nh);
  patternPublisher = it.advertise("dvs_calibration/pattern", 1);
  visualizationPublisher = it.advertise("dvs_calibration/visualization", 1);

  cameraInfoSubscriber = nh.subscribe("camera_info", 1, &MonoDvsCalibration::cameraInfoCallback, this);
  gotCameraInfo = false;
  cameraPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("dvs_calibration/pose", 1);
}

void MonoDvsCalibration::calibrate()
{
  // run camera calibration
  cv::Mat cameraMatrix, distCoeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  double reproj_error = cv::calibrateCamera(object_points, image_points, cv::Size(sensor_width, sensor_height),
                                            cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_FIX_K3);

  // update camera info
  new_camera_info.height = sensor_height;
  new_camera_info.width = sensor_width;
  new_camera_info.distortion_model = "plumb_bob";

  new_camera_info.D.clear();
  for (int i = 0; i < 5; i++)
    new_camera_info.D.push_back(distCoeffs.at<double>(i));
  for (int i = 0; i < 9; i++)
    new_camera_info.K[i] = cameraMatrix.at<double>(i);

  // send output
  std::ostringstream output;
  output << "Calibration result" << std::endl;
  output << "Reprojection error: " << std::setprecision(5) << reproj_error << std::endl;
  output << "Camera matrix (K):" << std::endl;
  for (int i = 0; i < 9; i++)
  {
    output << std::setfill(' ') << std::setw(6) << std::setprecision(5) << new_camera_info.K[i] << "  ";
    if (i%3 == 2)
      output << std::endl;
  }
  output << "Distortion coefficients (D):" << std::endl;
  for (int i = 0; i < 5; i++)
    output << std::setfill(' ') << std::setw(6) << std::setprecision(5) << new_camera_info.D[i] << "  ";
  output << std::endl;

  std_msgs::String output_msg;
  output_msg.data = output.str();
  calibrationOutputPublisher.publish(output_msg);
}

void MonoDvsCalibration::publishVisualizationImage(cv::Mat image)
{
  cv_bridge::CvImage cv_image;
  cv_image.encoding = "bgr8";
  cv_image.image = image.clone();

  visualizationPublisher.publish(cv_image.toImageMsg());
}

bool MonoDvsCalibration::setCameraInfo()
{
  sensor_msgs::SetCameraInfo srv;
  srv.request.camera_info = new_camera_info;
  return setCameraInfoClient.call(srv);
}

void MonoDvsCalibration::resetCalibration()
{
  transition_maps[mono_camera_id].reset_maps();
  calibration_running = false;
  object_points.clear();
  image_points.clear();
  num_detections = 0;
}

void MonoDvsCalibration::startCalibration()
{
  calibration_running = true;
  if (num_detections > 0)
  {
    calibrate();
  }
}

void MonoDvsCalibration::saveCalibration()
{
  if (setCameraInfo())
    ROS_INFO("Calibration saved successfully.");
  else
    ROS_ERROR("Error while saving calibration");
}

void MonoDvsCalibration::add_pattern(int id)
{
  // add detection
  image_points.push_back(transition_maps[id].pattern);
  object_points.push_back(world_pattern);
  num_detections++;

  // compute and publish camera pose if camera is calibrated
  if (gotCameraInfo)
  {
    cv::Mat rvec, tvec;
    cv::Mat cameraMatrix(3, 3, CV_64F);
    cv::Mat distCoeffs(1, 5, CV_64F);

    // convert to OpenCV
    for (int i = 0; i < 5; i++)
      distCoeffs.at<double>(i) = camera_info_external.D[i];
    for (int i = 0; i < 9; i++)
      cameraMatrix.at<double>(i) = camera_info_external.K[i];

    cv::solvePnP(world_pattern, transition_maps[id].pattern, cameraMatrix, distCoeffs, rvec, tvec);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "dvs";
    pose_msg.pose.position.x = tvec.at<double>(0);
    pose_msg.pose.position.y = tvec.at<double>(1);
    pose_msg.pose.position.z = tvec.at<double>(2);

    double angle = cv::norm(rvec);
    cv::normalize(rvec, rvec);
    pose_msg.pose.orientation.x = rvec.at<double>(0) * sin(angle/2.0);
    pose_msg.pose.orientation.y = rvec.at<double>(1) * sin(angle/2.0);
    pose_msg.pose.orientation.z = rvec.at<double>(2) * sin(angle/2.0);
    pose_msg.pose.orientation.w = cos(angle/2.0);

    cameraPosePublisher.publish(pose_msg);
  }
}

void MonoDvsCalibration::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  gotCameraInfo = true;
  camera_info_external = *msg;
}

void MonoDvsCalibration::update_visualization(int id)
{
  publishVisualizationImage(transition_maps[id].get_visualization_image());
}

} // namespace

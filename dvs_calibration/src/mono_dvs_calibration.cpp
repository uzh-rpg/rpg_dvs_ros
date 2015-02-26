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
}

void MonoDvsCalibration::calibrate()
{
  // run camera calibration
  cv::Mat cameraMatrix, distCoeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  double reproj_error = cv::calibrateCamera(object_points, image_points, cv::Size(sensor_width, sensor_height),
                                            cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_FIX_K3);

  // update camera info
  cameraInfo.height = sensor_height;
  cameraInfo.width = sensor_width;
  cameraInfo.distortion_model = "plumb_bob";

  cameraInfo.D.clear();
  for (int i = 0; i < 5; i++)
    cameraInfo.D.push_back(distCoeffs.at<double>(i));
  for (int i = 0; i < 9; i++)
    cameraInfo.K[i] = cameraMatrix.at<double>(i);

  // send output
  std::ostringstream output;
  output << "Calibration result" << std::endl;
  output << "Reprojection error: " << std::setprecision(5) << reproj_error << std::endl;
  output << "Camera matrix (K):" << std::endl;
  for (int i = 0; i < 9; i++)
  {
    output << std::setfill(' ') << std::setw(6) << std::setprecision(5) << cameraInfo.K[i] << "  ";
    if (i%3 == 2)
      output << std::endl;
  }
  output << "Distortion coefficients (D):" << std::endl;
  for (int i = 0; i < 5; i++)
    output << std::setfill(' ') << std::setw(6) << std::setprecision(5) << cameraInfo.D[i] << "  ";
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
  srv.request.camera_info = cameraInfo;
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
  setCameraInfo();
}

void MonoDvsCalibration::add_pattern(int id)
{
  // add detection
  image_points.push_back(transition_maps[id].pattern);
  object_points.push_back(world_pattern);
  num_detections++;
}

void MonoDvsCalibration::update_visualization(int id)
{
  publishVisualizationImage(transition_maps[id].get_visualization_image());
}

} // namespace

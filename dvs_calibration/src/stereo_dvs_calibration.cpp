// This file is part of DVS-ROS - the RPG DVS ROS Package

#include "dvs_calibration/stereo_dvs_calibration.h"

namespace dvs_calibration {

StereoDvsCalibration::StereoDvsCalibration()
{
  got_camera_info_left_ = false;
  got_camera_info_right_ = false;

  has_left_buffer_ = false;
  has_right_buffer_ = false;

  calibration_running_ = false;

  set_camera_info_left_client_ = nh_.serviceClient<sensor_msgs::SetCameraInfo>("set_camera_info_left");
  set_camera_info_right_client_ = nh_.serviceClient<sensor_msgs::SetCameraInfo>("set_camera_info_right");
  camera_info_left_sub_ = nh_.subscribe("camera_info_left", 1, &StereoDvsCalibration::cameraInfoLeftCallback, this);
  camera_info_right_sub_ = nh_.subscribe("camera_info_right", 1, &StereoDvsCalibration::cameraInfoRightCallback, this);

  // add transition map
  transition_maps_.insert(std::pair<int, TransitionMap>(left_camera_id, TransitionMap(params_)));
  event_left_sub_ = nh_.subscribe<dvs_msgs::EventArray>("events_left", 1,
                                                       boost::bind(&StereoDvsCalibration::eventsCallback, this, _1, left_camera_id));
  transition_maps_.insert(std::pair<int, TransitionMap>(right_camera_id, TransitionMap(params_)));
  event_right_sub_ = nh_.subscribe<dvs_msgs::EventArray>("events_right", 1,
                                                       boost::bind(&StereoDvsCalibration::eventsCallback, this, _1, right_camera_id));

  image_transport::ImageTransport it(nh_);
  visualization_left_pub_ = it.advertise("dvs_calibration/visualization_left", 1);
  visualization_right_pub_ = it.advertise("dvs_calibration/visualization_right", 1);
}

void StereoDvsCalibration::calibrate()
{
  cv::Mat distCoeffsLeft(1, 5, CV_64F);
  cv::Mat distCoeffsRight(1, 5, CV_64F);
  cv::Mat cameraMatrixLeft(3, 3, CV_64F);
  cv::Mat cameraMatrixRight(3, 3, CV_64F);

  // convert to OpenCV
  for (int i = 0; i < 5; i++) {
    distCoeffsLeft.at<double>(i) = camera_info_left_.D[i];
    distCoeffsRight.at<double>(i) = camera_info_right_.D[i];
  }
  for (int i = 0; i < 9; i++) {
    cameraMatrixLeft.at<double>(i) = camera_info_left_.K[i];
    cameraMatrixRight.at<double>(i) = camera_info_right_.K[i];
  }

  // call stereo calibration
  cv::Mat R, T, E, F;
  double reproj_error = cv::stereoCalibrate(object_points_, image_points_left_, image_points_right_,
                                            cameraMatrixLeft, distCoeffsLeft, cameraMatrixRight, distCoeffsRight,
                                            cv::Size(sensor_width_, sensor_height_),
                                            R, T, E, F);

  cv::Mat R1, R2, P1, P2, Q;
  cv::stereoRectify(cameraMatrixLeft, distCoeffsLeft, cameraMatrixRight, distCoeffsRight,
                    cv::Size(sensor_width_, sensor_height_), R, T, R1, R2, P1, P2, Q);

  // update camera messages
  for (int i = 0; i < 9; i++) {
    camera_info_left_.R[i] = R1.at<double>(i);
    camera_info_right_.R[i] = R2.at<double>(i);
  }
  for (int i = 0; i < 12; i++) {
    camera_info_left_.P[i] = P1.at<double>(i);
    camera_info_right_.P[i] = P2.at<double>(i);
  }

  // send output
  std::ostringstream output;
  output << "Calibration result" << std::endl;
  output << "Reprojection error: " << std::setprecision(5) << reproj_error << std::endl;

  output << "R:" << std::endl;
  for (int i = 0; i < 9; i++)
  {
    output << std::setfill(' ') << std::setw(6) << std::setprecision(5) << R.at<double>(i) << "  ";
    if (i%3 == 2)
      output << std::endl;
  }
  output << "T:" << std::endl;
  for (int i = 0; i < 3; i++)
  {
    output << std::setfill(' ') << std::setw(6) << std::setprecision(5) << T.at<double>(i) << "  ";
  }

  std_msgs::String output_msg;
  output_msg.data = output.str();
  calibration_output_pub_.publish(output_msg);
}

void StereoDvsCalibration::publishVisualizationImage(cv::Mat image, int id)
{
  cv_bridge::CvImage cv_image;
  cv_image.encoding = "bgr8";
  cv_image.image = image.clone();

  if (id == left_camera_id) {
    visualization_left_pub_.publish(cv_image.toImageMsg());
  }
  else if (id == right_camera_id) {
    visualization_right_pub_.publish(cv_image.toImageMsg());
  }
}

bool StereoDvsCalibration::setCameraInfo()
{
  sensor_msgs::SetCameraInfo srvLeft, srvRight;

  srvLeft.request.camera_info = camera_info_left_;
  set_camera_info_left_client_.call(srvLeft);

  srvRight.request.camera_info = camera_info_right_;
  set_camera_info_right_client_.call(srvRight);

  return true;
}

void StereoDvsCalibration::cameraInfoLeftCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if (got_camera_info_left_)
    return;

  got_camera_info_left_ = true;
  camera_info_left_ = *msg;
}

void StereoDvsCalibration::cameraInfoRightCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if (got_camera_info_right_)
    return;

  got_camera_info_right_ = true;
  camera_info_right_ = *msg;
}

void StereoDvsCalibration::resetCalibration()
{
  transition_maps_[left_camera_id].reset_maps();
  transition_maps_[right_camera_id].reset_maps();
  calibration_running_ = false;
  object_points_.clear();
  image_points_left_.clear();
  image_points_right_.clear();
  num_detections_ = 0;
  has_left_buffer_ = false;
  has_right_buffer_ = false;
}

void StereoDvsCalibration::startCalibration()
{
  calibration_running_ = true;
  if (num_detections_ > 0)
  {
    calibrate();
  }
}

void StereoDvsCalibration::saveCalibration()
{
  setCameraInfo();
}

void StereoDvsCalibration::addPattern(int id)
{
  const ros::Duration stereo_max_buffer_time_difference(0.3);

  if (got_camera_info_left_ && got_camera_info_right_) {
    if (id == left_camera_id) {
      ROS_INFO("pattern from left camera");
      if (has_right_buffer_) {
        if (ros::Time::now() - buffer_time_ < stereo_max_buffer_time_difference) {
          addStereoPattern(transition_maps_[left_camera_id].pattern, image_point_buffer_);
          ROS_INFO("Added detection!");
        }
        else {
          ROS_INFO("Time difference too large, did not add...");
        }
      }
      else {
        // add to buffer
        buffer_time_ = ros::Time::now();
        image_point_buffer_ = transition_maps_[left_camera_id].pattern;
        has_left_buffer_ = true;
      }
    }
    else if (id == right_camera_id) {
      ROS_INFO("pattern from right camera");
      if (has_left_buffer_) {
        if (ros::Time::now() - buffer_time_ < stereo_max_buffer_time_difference) {
          addStereoPattern(image_point_buffer_, transition_maps_[right_camera_id].pattern);
          ROS_INFO("Added detection!");
        }
        else {
          ROS_INFO("Time difference too large, did not add...");
        }
      }
      else {
        // add to buffer
        buffer_time_ = ros::Time::now();
        image_point_buffer_ = transition_maps_[right_camera_id].pattern;
        has_right_buffer_ = true;
      }
    }
  }
  else {
    ROS_WARN("Did not receive camera info message yet.");
  }
}

void StereoDvsCalibration::addStereoPattern(std::vector<cv::Point2f> left, std::vector<cv::Point2f> right)
{
  // add detection
  image_points_left_.push_back(left);
  image_points_right_.push_back(right);
  object_points_.push_back(world_pattern_);
  num_detections_++;
}

void StereoDvsCalibration::updateVisualization(int id)
{
  publishVisualizationImage(transition_maps_[id].get_visualization_image(), id);
}

} // namespace

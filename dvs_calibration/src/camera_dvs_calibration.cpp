// This file is part of DVS-ROS - the RPG DVS ROS Package

#include "dvs_calibration/camera_dvs_calibration.h"

namespace dvs_calibration {

CameraDvsCalibration::CameraDvsCalibration()
{
  got_standard_camera_info_ = false;
  got_dvs_camera_info_ = false;

  calibration_running_ = false;

  last_detection_camera_time_ = ros::Time::now() - ros::Duration(10*max_time_difference_);

  // parameters
  ros::NodeHandle nh_private("~");
  nh_private.param("image_led_threshold", image_led_threshold_, 250);
  nh_private.param("max_time_difference", max_time_difference_, 0.1);
  nh_private.param("fix_intrinsics", fix_intrinsics_, true);

  ROS_INFO("%s: Parameters:", ros::this_node::getName().c_str());
  ROS_INFO("%s:   image_led_threshold = %d", ros::this_node::getName().c_str(), image_led_threshold_);
  ROS_INFO("%s:   max_time_difference = %f", ros::this_node::getName().c_str(), max_time_difference_);
  ROS_INFO("%s:   fix_intrinsics = %s", ros::this_node::getName().c_str(), fix_intrinsics_ ? "true" : "false");


  // publishers
  image_transport::ImageTransport it(nh_);
  camera_visualization_pub_ = it.advertise("camera_dvs_calibration/visualization_camera", 1);
  dvs_visualization_pub_ = it.advertise("camera_dvs_calibration/visualization_dvs", 1);

  set_standard_camera_info_client_ = nh_.serviceClient<sensor_msgs::SetCameraInfo>("set_standard_camera_info");
  set_dvs_camera_info_client_ = nh_.serviceClient<sensor_msgs::SetCameraInfo>("set_dvs_camera_info");
  standard_camera_info_sub_ = nh_.subscribe("standard_camera_info", 1, &CameraDvsCalibration::standardCameraInfoCallback, this);
  dvs_camera_info_sub_ = nh_.subscribe("dvs_camera_info", 1, &CameraDvsCalibration::dvsCameraInfoCallback, this);

  // add transition map
  transition_maps_.insert(std::pair<int, TransitionMap>(right_camera_id, TransitionMap(params_)));
  event_sub_ = nh_.subscribe<dvs_msgs::EventArray>("events", 1,
                                                       boost::bind(&CameraDvsCalibration::eventsCallback, this, _1, right_camera_id));

  image_sub_ = nh_.subscribe("image", 1, &CameraDvsCalibration::imageCallback, this);



}

void CameraDvsCalibration::calibrate()
{
  cv::Mat dist_coeffs_camera(1, 5, CV_64F);
  cv::Mat dist_coeffs_dvs(1, 5, CV_64F);
  cv::Mat camera_matrix_camera(3, 3, CV_64F);
  cv::Mat camera_matrix_dvs(3, 3, CV_64F);

  // convert to OpenCV
  for (int i = 0; i < 5; i++)
  {
    dist_coeffs_camera.at<double>(i) = standard_camera_info_.D[i];
    dist_coeffs_dvs.at<double>(i) = dvs_camera_info_.D[i];
  }
  for (int i = 0; i < 9; i++)
  {
    camera_matrix_camera.at<double>(i) = standard_camera_info_.K[i];
    camera_matrix_dvs.at<double>(i) = dvs_camera_info_.K[i];
  }

  // call stereo calibration
  cv::Mat R, T, E, F;
  int flags = fix_intrinsics_ ? cv::CALIB_FIX_INTRINSIC : cv::CALIB_USE_INTRINSIC_GUESS;

  cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6);
#if CV_MAJOR_VERSION == 2
  double reproj_error = cv::stereoCalibrate(object_points_, image_points_camera_, image_points_dvs_,
                                            camera_matrix_camera, dist_coeffs_camera, camera_matrix_dvs, dist_coeffs_dvs,
                                            cv::Size(standard_camera_info_.width, standard_camera_info_.height),
                                            R, T, E, F, term_crit, flags);
#elif CV_MAJOR_VERSION == 3 || CV_MAJOR_VERSION == 4
double reproj_error = cv::stereoCalibrate(object_points_, image_points_camera_, image_points_dvs_,
                                          camera_matrix_camera, dist_coeffs_camera, camera_matrix_dvs, dist_coeffs_dvs,
                                          cv::Size(standard_camera_info_.width, standard_camera_info_.height),
                                          R, T, E, F, flags, term_crit);
#endif
  cv::Mat R1, R2, P1, P2, Q;
  cv::stereoRectify(camera_matrix_camera, dist_coeffs_camera, camera_matrix_dvs, dist_coeffs_dvs,
                    cv::Size(standard_camera_info_.width, standard_camera_info_.height), R, T, R1, R2, P1, P2, Q);

  // update camera messages
  new_standard_camera_info_ = standard_camera_info_;
  new_dvs_camera_info_ = dvs_camera_info_;

  for (int i = 0; i < 9; i++)
  {
    new_standard_camera_info_.R[i] = R1.at<double>(i);
    new_dvs_camera_info_.R[i] = R2.at<double>(i);
  }
  for (int i = 0; i < 12; i++)
  {
    new_standard_camera_info_.P[i] = P1.at<double>(i);
    new_dvs_camera_info_.P[i] = P2.at<double>(i);
  }

  if (!fix_intrinsics_)
  {
    new_standard_camera_info_.D.clear();
    new_dvs_camera_info_.D.clear();

    for (int i = 0; i < 5; i++)
    {
      new_standard_camera_info_.D.push_back(dist_coeffs_camera.at<double>(i));
      new_dvs_camera_info_.D.push_back(dist_coeffs_dvs.at<double>(i));
    }
    for (int i = 0; i < 9; i++)
    {
      new_standard_camera_info_.K[i] = camera_matrix_camera.at<double>(i);
      new_dvs_camera_info_.K[i] = camera_matrix_dvs.at<double>(i);
    }
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
  output << std::endl;

  if (fix_intrinsics_)
  {
    output << "Save calibration will add the rectification transform (rotation matrix) and projection matrix for both cameras." << std::endl;
  }
  else
  {
    output << "Save calibration will update all camera parameters, including the camera matrix and distortion coefficients." << std::endl;
  }

  std_msgs::String output_msg;
  output_msg.data = output.str();
  calibration_output_pub_.publish(output_msg);
}

void CameraDvsCalibration::publishVisualizationImage(cv::Mat image, int id)
{
  cv_bridge::CvImage cv_image;
  cv_image.encoding = "bgr8";
  cv_image.image = image.clone();

  dvs_visualization_pub_.publish(cv_image.toImageMsg());
}

bool CameraDvsCalibration::setCameraInfo()
{
  sensor_msgs::SetCameraInfo srv_camera, srv_dvs;

  srv_camera.request.camera_info = new_standard_camera_info_;
  set_standard_camera_info_client_.call(srv_camera);

  srv_dvs.request.camera_info = new_dvs_camera_info_;
  set_dvs_camera_info_client_.call(srv_dvs);

  return true;
}

void CameraDvsCalibration::standardCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  got_standard_camera_info_ = true;
  standard_camera_info_ = *msg;
}

void CameraDvsCalibration::dvsCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  got_dvs_camera_info_ = true;
  dvs_camera_info_ = *msg;
}

void CameraDvsCalibration::imageCallback(const sensor_msgs::Image::ConstPtr &msg)
{
  if (!got_standard_camera_info_)
  {
    ROS_WARN("%s: Camera info not yet received.", ros::this_node::getName().c_str());
  }

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // threshodl and invert
  cv::Mat thresholded_image;
  cv::threshold(cv_ptr->image, thresholded_image, image_led_threshold_, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<cv::Point2f> centers;

  cv::findContours( thresholded_image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); //Find the Contour BLOBS
  for( int i = 0; i < contours.size(); i++ )
  {
    cv::Moments mu = moments( cv::Mat(contours[i]), false );
    cv::Point2f center = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00);
    if (center.x > 10 && center.y > 10)
      centers.push_back(center);
  }

  // find centers
  std::vector<cv::Point2f> centers_good;
  cv::Size patternsize(params_.dots_w, params_.dots_h);
  CirclesGridClusterFinder grid(false);
  grid.findGrid(centers, patternsize, centers_good);

  if (centers_good.size() == params_.dots_w * params_.dots_h)
  {
    camera_image_point_buffer_ = centers_good;
    last_detection_camera_time_ = ros::Time::now();
  }

  // visualize checkerboard
  cv_bridge::CvImage cv_ptr_visu;
  cv_ptr->image.copyTo(cv_ptr_visu.image);
  cv_ptr_visu.encoding = "bgr8";
  cvtColor(cv_ptr_visu.image, cv_ptr_visu.image, CV_GRAY2RGB);

  if (centers_good.size() == params_.dots_w * params_.dots_h)
  {
    cv::drawChessboardCorners(cv_ptr_visu.image, cv::Size(params_.dots_w, params_.dots_h), cv::Mat(centers_good), true);
  }

  camera_visualization_pub_.publish(cv_ptr_visu.toImageMsg());
}

void CameraDvsCalibration::resetCalibration()
{
  transition_maps_[right_camera_id].reset_maps();
  calibration_running_ = false;

  object_points_.clear();
  image_points_camera_.clear();
  image_points_dvs_.clear();
  num_detections_ = 0;
}

void CameraDvsCalibration::startCalibration()
{
  calibration_running_ = true;
  if (num_detections_ > 0)
  {
    calibrate();
  }
}

void CameraDvsCalibration::saveCalibration()
{
  setCameraInfo();
}

void CameraDvsCalibration::addPattern(int id)
{
  if (got_dvs_camera_info_) {
    if (ros::Time::now() - last_detection_camera_time_ < ros::Duration(max_time_difference_)) {
      add_stereo_pattern(camera_image_point_buffer_, transition_maps_[right_camera_id].pattern);
      ROS_INFO("%s: Added detection!", ros::this_node::getName().c_str());
    }
    else {
      ROS_INFO("%s: Time difference too large, did not add...", ros::this_node::getName().c_str());
    }
  }
  else {
    ROS_WARN("%s: Did not receive camera info message yet.", ros::this_node::getName().c_str());
  }
}

void CameraDvsCalibration::add_stereo_pattern(std::vector<cv::Point2f> camera_image_points, std::vector<cv::Point2f> dvs_image_points)
{
  // add detection
  image_points_camera_.push_back(camera_image_points);
  image_points_dvs_.push_back(dvs_image_points);
  object_points_.push_back(world_pattern_);
  num_detections_++;

}

void CameraDvsCalibration::updateVisualization(int id)
{
  publishVisualizationImage(transition_maps_[id].get_visualization_image(), id);
}

} // namespace

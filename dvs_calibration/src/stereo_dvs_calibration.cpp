#include "dvs_calibration/stereo_dvs_calibration.h"

StereoDvsCalibration::StereoDvsCalibration()
{
  gotCameraInfoLeft = false;
  gotCameraInfoRight = false;

  has_left_buffer = false;
  has_right_buffer = false;

  calibration_running = false;

  setCameraInfoLeftClient = nh.serviceClient<sensor_msgs::SetCameraInfo>("set_camera_info_left");
  setCameraInfoRightClient = nh.serviceClient<sensor_msgs::SetCameraInfo>("set_camera_info_right");
  cameraInfoLeftSubscriber = nh.subscribe("camera_info_left", 1, &StereoDvsCalibration::cameraInfoLeftCallback, this);
  cameraInfoRightSubscriber = nh.subscribe("camera_info_right", 1, &StereoDvsCalibration::cameraInfoRightCallback, this);

  // add transition map
  const int camera_left_id = 1;
  const int camera_right_id = 2;
  transition_maps.insert(std::pair<int, TransitionMap>(camera_left_id, TransitionMap()));
  eventLeftSubscriber = nh.subscribe<dvs_msgs::EventArray>("events_left", 1,
                                                       boost::bind(&StereoDvsCalibration::eventsCallback, this, _1, camera_left_id));
  transition_maps.insert(std::pair<int, TransitionMap>(camera_right_id, TransitionMap()));
  eventRightSubscriber = nh.subscribe<dvs_msgs::EventArray>("events_right", 1,
                                                       boost::bind(&StereoDvsCalibration::eventsCallback, this, _1, camera_right_id));


  cameraInfoPublisher = nh.advertise<sensor_msgs::CameraInfo>("dvs_calibration/camera_info", 1);
  reprojectionErrorPublisher = nh.advertise<std_msgs::Float64>("dvs_calibration/calibration_reprojection_error", 1);
  cameraPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("dvs_calibration/pose", 1);

  image_transport::ImageTransport it(nh);
  visualizationLeftPublisher = it.advertise("dvs_calibration/visualization_left", 1);
  visualizationRightPublisher = it.advertise("dvs_calibration/visualization_right", 1);
}

void StereoDvsCalibration::calibrate()
{
  cv::Mat distCoeffsLeft(1, 5, CV_64F);
  cv::Mat distCoeffsRight(1, 5, CV_64F);
  cv::Mat cameraMatrixLeft(3, 3, CV_64F);
  cv::Mat cameraMatrixRight(3, 3, CV_64F);

  // convert to OpenCV
  for (int i = 0; i < 5; i++) {
    distCoeffsLeft.at<double>(i) = cameraInfoLeft.D[i];
    distCoeffsRight.at<double>(i) = cameraInfoRight.D[i];
  }
  for (int i = 0; i < 9; i++) {
    cameraMatrixLeft.at<double>(i) = cameraInfoLeft.K[i];
    cameraMatrixRight.at<double>(i) = cameraInfoRight.K[i];
  }

  cv::Mat R, T, E, F;
  double reproj_error = cv::stereoCalibrate(object_points, image_points_left, image_points_right,
                                            cameraMatrixLeft, distCoeffsLeft, cameraMatrixRight, distCoeffsRight,
                                            cv::Size(sensor_width, sensor_height),
                                            R, T, E, F);

  cv::Mat R1, R2, P1, P2, Q;
  cv::stereoRectify(cameraMatrixLeft, distCoeffsLeft, cameraMatrixRight, distCoeffsRight,
                    cv::Size(sensor_width, sensor_height), R, T, R1, R2, P1, P2, Q);

  // update camera messages
  for (int i = 0; i < 9; i++) {
    cameraInfoLeft.R[i] = R1.at<double>(i);
    cameraInfoRight.R[i] = R2.at<double>(i);
  }
  for (int i = 0; i < 12; i++) {
    cameraInfoLeft.P[i] = P1.at<double>(i);
    cameraInfoRight.P[i] = P2.at<double>(i);
  }

  //  cameraInfoPublisher.publish(msg);
//  cameraInfo = msg;

  std_msgs::Float64 msg2;
  msg2.data = reproj_error;
  reprojectionErrorPublisher.publish(msg2);
}

void StereoDvsCalibration::publishVisualizationImage(cv::Mat image, int id)
{
  cv_bridge::CvImage cv_image;
  cv_image.encoding = "bgr8";
  cv_image.image = image.clone();

  const int camera_left_id = 1;
  const int camera_right_id = 2;
  if (id == camera_left_id) {
    visualizationLeftPublisher.publish(cv_image.toImageMsg());
  }
  else if (id == camera_right_id) {
    visualizationRightPublisher.publish(cv_image.toImageMsg());
  }
}

bool StereoDvsCalibration::setCameraInfo()
{
  sensor_msgs::SetCameraInfo srvLeft, srvRight;

  srvLeft.request.camera_info = cameraInfoLeft;
  setCameraInfoLeftClient.call(srvLeft);

  srvRight.request.camera_info = cameraInfoRight;
  setCameraInfoRightClient.call(srvRight);

  return true;
}

void StereoDvsCalibration::cameraInfoLeftCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if (gotCameraInfoLeft)
    return;

  gotCameraInfoLeft = true;
  cameraInfoLeft = *msg;
}

void StereoDvsCalibration::cameraInfoRightCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if (gotCameraInfoRight)
    return;

  gotCameraInfoRight = true;
  cameraInfoRight = *msg;
}

void StereoDvsCalibration::resetCalibration()
{
  const int camera_left_id = 1;
  const int camera_right_id = 2;
  transition_maps[camera_left_id].reset_maps();
  transition_maps[camera_right_id].reset_maps();
  calibration_running = false;
  object_points.clear();
  image_points_left.clear();
  image_points_right.clear();
  num_detections = 0;
  has_left_buffer = false;
  has_right_buffer = false;
}

void StereoDvsCalibration::startCalibration()
{
  calibration_running = true;
  if (num_detections > 0)
  {
    calibrate();
  }
}

void StereoDvsCalibration::saveCalibration()
{
  setCameraInfo();
}

void StereoDvsCalibration::add_pattern(int id)
{
  const int camera_left_id = 1;
  const int camera_right_id = 2;
  const ros::Duration stereo_max_buffer_time_difference(0.3);

  if (gotCameraInfoLeft && gotCameraInfoRight) {
    if (id == camera_left_id) {
      ROS_INFO("pattern from left camera");
      if (has_right_buffer) {
        if (ros::Time::now() - buffer_time < stereo_max_buffer_time_difference) {
          add_stereo_pattern(transition_maps[camera_left_id].pattern, image_point_buffer);
          ROS_INFO("Added detection!");
        }
        else {
          ROS_INFO("Time difference too large, did not add...");
        }
      }
      else {
        // add to buffer
        buffer_time = ros::Time::now();
        image_point_buffer = transition_maps[camera_left_id].pattern;
        has_left_buffer = true;
      }
    }
    else if (id == camera_right_id) {
      ROS_INFO("pattern from right camera");
      if (has_left_buffer) {
        if (ros::Time::now() - buffer_time < stereo_max_buffer_time_difference) {
          add_stereo_pattern(image_point_buffer, transition_maps[camera_right_id].pattern);
          ROS_INFO("Added detection!");
        }
        else {
          ROS_INFO("Time difference too large, did not add...");
        }
      }
      else {
        // add to buffer
        buffer_time = ros::Time::now();
        image_point_buffer = transition_maps[camera_right_id].pattern;
        has_right_buffer = true;
      }
    }
  }
  else {
    ROS_WARN("Did not receive camera info message yet.");
  }
}

void StereoDvsCalibration::add_stereo_pattern(std::vector<cv::Point2f> left, std::vector<cv::Point2f> right)
{
  // add detection
  image_points_left.push_back(left);
  image_points_right.push_back(right);
  object_points.push_back(world_pattern);
  num_detections++;
}

void StereoDvsCalibration::update_visualization(int id)
{
  publishVisualizationImage(transition_maps[id].get_visualization_image(), id);
}

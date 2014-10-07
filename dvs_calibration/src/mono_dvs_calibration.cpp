#include "dvs_calibration/mono_dvs_calibration.h"

MonoDvsCalibration::MonoDvsCalibration()
{
  gotCameraInfo = false;

  calibration_running = false;
  num_detections = 0;

  setCameraInfoClient = nh.serviceClient<sensor_msgs::SetCameraInfo>("set_camera_info");
  cameraInfoSubscriber = nh.subscribe("camera_info", 1, &MonoDvsCalibration::cameraInfoCallback, this);


  eventSubscriber = nh.subscribe<dvs_msgs::EventArray>("/dvs_right/events", 1,
                                                       boost::bind(&MonoDvsCalibration::eventsCallback, this, _1, 1));

  detectionPublisher = nh.advertise<std_msgs::Int32>("dvs_calibration/pattern_detections", 1);
  cameraInfoPublisher = nh.advertise<sensor_msgs::CameraInfo>("dvs_calibration/camera_info", 1);
  reprojectionErrorPublisher = nh.advertise<std_msgs::Float64>("dvs_calibration/calibration_reprojection_error", 1);
  cameraPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("dvs_calibration/pose", 1);

  image_transport::ImageTransport it(nh);
  patternPublisher = it.advertise("dvs_calibration/pattern", 1);
  visualizationPublisher = it.advertise("dvs_calibration/visualization", 1);
}

void MonoDvsCalibration::calibrate()
{
  cv::Mat cameraMatrix, distCoeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  double reproj_error = cv::calibrateCamera(object_points, image_points, cv::Size(sensor_width, sensor_height),
                                            cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_FIX_K3);

  sensor_msgs::CameraInfo msg;

  msg.height = sensor_height;
  msg.width = sensor_width;
  msg.distortion_model = "plumb_bob";

  for (int i = 0; i < 5; i++)
    msg.D.push_back(distCoeffs.at<double>(i));
  for (int i = 0; i < 9; i++)
    msg.K[i] = cameraMatrix.at<double>(i);

  cameraInfoPublisher.publish(msg);
  cameraInfo = msg;

  std_msgs::Float64 msg2;
  msg2.data = reproj_error;
  reprojectionErrorPublisher.publish(msg2);
}

void MonoDvsCalibration::publishVisualizationImage(std::vector<cv::Point2f> pattern)
{
//  cv_bridge::CvImage cv_image;
//  cv_image.encoding = "bgr8";
//  cv_image.image = cv::Mat(sensor_height, sensor_width, CV_8UC3);
//  cv_image.image = cv::Scalar(255, 255, 255);
//  for (int i = 0; i < sensor_width; i++)
//  {
//    for (int j = 0; j < sensor_height; j++)
//    {
//      int value = 255.0 - ((double)transition_sum_map[i][j]) / ((double)enough_transitions_threshold) * 255.0;
//      cv_image.image.at<cv::Vec3b>(j, i) = cv::Vec3b(value, value, value);
//    }
//  }
//
//  if (pattern.size() == dots * dots)
//    cv::drawChessboardCorners(cv_image.image, cv::Size(dots, dots), cv::Mat(pattern), true);

//  visualizationPublisher.publish(cv_image.toImageMsg());
}

void MonoDvsCalibration::publishPatternImage(cv::Mat image)
{
  cv_bridge::CvImage cv_image;
  cv_image.encoding = "mono8";
  cv_image.image = image.clone();
  patternPublisher.publish(cv_image.toImageMsg());
}
//
//void MonoDvsCalibration::resetIntrinsicCalibration()
//{
//  publishPatternImage(pattern.get_intrinsic_calibration_pattern());
//  object_points.clear();
//  image_points.clear();
//  num_detections = 0;
//}
//
//bool MonoDvsCalibration::startCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
//{
//  calibration_running = true;
//  calibrate();
//  return true;
//}
//
//bool MonoDvsCalibration::resetCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
//{
//  calibration_running = false;
//  resetIntrinsicCalibration();
//  return true;
//}


bool MonoDvsCalibration::setCameraInfo()
{
  sensor_msgs::SetCameraInfo srv;
  srv.request.camera_info = cameraInfo;
  return setCameraInfoClient.call(srv);
}

void MonoDvsCalibration::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  gotCameraInfo = true;
  cameraInfo = *msg;
}

void MonoDvsCalibration::resetCalibration()
{
}

void MonoDvsCalibration::startCalibration()
{
}

void MonoDvsCalibration::saveCalibration()
{
  setCameraInfo();
}

#include "dvs_calibration/dvs_calibration.h"

DvsCalibration::DvsCalibration()
{
  calibration_running = false;
  num_detections = 0;

  startCalibrationService = nh.advertiseService("dvs_calibration/start", &DvsCalibration::startCalibrationCallback, this);
  saveCalibrationService = nh.advertiseService("dvs_calibration/save", &DvsCalibration::saveCalibrationCallback, this);
  resetService = nh.advertiseService("dvs_calibration/reset", &DvsCalibration::resetCallback, this);

  setCameraInfoClient = nh.serviceClient<sensor_msgs::SetCameraInfo>("/set_camera_info");
  cameraInfoSubscriber = nh.subscribe("dvs/camera_info", 1, &DvsCalibration::cameraInfoCallback, this);

  eventSubscriber = nh.subscribe("dvs/events", 10, &DvsCalibration::eventsCallback, this);
  detectionPublisher = nh.advertise<std_msgs::Int32>("dvs_calibration/pattern_detections", 1);
  cameraInfoPublisher = nh.advertise<sensor_msgs::CameraInfo>("dvs_calibration/camera_info", 1);
  reprojectionErrorPublisher = nh.advertise<std_msgs::Float64>("dvs_calibration/calibration_reprojection_error", 1);
  cameraPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("dvs_calibration/pose", 1);

  image_transport::ImageTransport it(nh);
  patternPublisher = it.advertise("dvs_calibration/pattern", 1);
  visualizationPublisher = it.advertise("dvs_calibration/visualization", 1);
}

void DvsCalibration::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // don't take new events while running
  if (calibration_running)
    return;

  for (int i = 0; i < msg->events.size(); ++i)
  {
    if (msg->events[i].polarity == true)
    {
      last_off_map[msg->events[i].x][msg->events[i].y] = msg->events[i].time;
    }
    else
    {
      int delta_t = msg->events[i].time - last_off_map[msg->events[i].x][msg->events[i].y];
      if (delta_t < blinking_time_us + blinking_time_tolerance && delta_t > blinking_time_us - blinking_time_tolerance)
        transition_sum_map[msg->events[i].x][msg->events[i].y]++;
    }
  }

  // check if enough transition events
  bool enough_transitions = false;
  for (int i = 0; i < sensor_width; i++)
  {
    for (int j = 0; j < sensor_height; j++)
    {
      if (transition_sum_map[i][j] > enough_transitions_threshold)
        enough_transitions = true;
    }
  }

  if (enough_transitions)
  {
    std::vector<cv::Point2f> centers = findPattern();
    if (centers.size() == dots * dots)
    {
      publishVisualizationImage(centers);

      image_points.push_back(centers);

      std::vector<cv::Point3f> world_pattern;
      for (int i = 0; i < dots; i++)
      {
        for (int j = 0; j < dots; j++)
        {
          world_pattern.push_back(cv::Point3f(i * dot_distance , j * dot_distance , 0.0));
        }
      }

      object_points.push_back(world_pattern);

      num_detections++;

      // if we have camera info, also publish pose
      if (gotCameraInfo)
      {
        cv::Mat rvec, tvec;
        cv::Mat cameraMatrix(3, 3, CV_64F);
        cv::Mat distCoeffs(1, 5, CV_64F);

        // convert to OpenCV
        for (int i = 0; i < 5; i++)
          distCoeffs.at<double>(i) = cameraInfo.D[i];
        for (int i = 0; i < 9; i++)
          cameraMatrix.at<double>(i) = cameraInfo.K[i];

        cv::solvePnP(world_pattern, centers, cameraMatrix, distCoeffs, rvec, tvec);

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "dvs";
        pose_msg.pose.position.x = tvec.at<double>(0);
        pose_msg.pose.position.y = tvec.at<double>(1);
        pose_msg.pose.position.y = tvec.at<double>(2);

        double angle = cv::norm(rvec);
        cv::normalize(rvec, rvec);
        pose_msg.pose.orientation.x = rvec.at<double>(0) * sin(angle/2.0);
        pose_msg.pose.orientation.y = rvec.at<double>(1) * sin(angle/2.0);
        pose_msg.pose.orientation.z = rvec.at<double>(2) * sin(angle/2.0);
        pose_msg.pose.orientation.w = cos(angle/2.0);

        cameraPosePublisher.publish(pose_msg);
      }

      // send status
      std_msgs::Int32 msg;
      msg.data = num_detections;
      detectionPublisher.publish(msg);

      // update last detection time
      last_pattern_found = ros::Time::now();
    }

    reset_maps();
  }
  else
  {
    std::vector<cv::Point2f> centers;
    publishVisualizationImage(centers);
  }

  if (ros::Time::now() - last_pattern_found > ros::Duration(pattern_search_timeout)) {
    reset_maps();
    last_pattern_found = ros::Time::now();
  }
}

void DvsCalibration::reset_maps()
{
  for (int i = 0; i < sensor_width; i++)
  {
    for (int j = 0; j < sensor_height; j++)
    {
      last_on_map[i][j] = 0;
      last_off_map[i][j] = 0;
      transition_sum_map[i][j] = 0;
    }
  }
}

std::vector<cv::Point2f> DvsCalibration::findPattern()
{
//  cv::Size patternsize(dots, dots); //number of centers
//  std::vector<cv::Point2f> centers; //this will be filled by the detected centers
//  std::vector<cv::Point2f> centers_tmp; //this will be filled by the detected centers
//  std::vector<int> center_count;

//  std::list<cv::Point> points;
//  std::vector<std::list<cv::Point> > clusters;

  std::list<PointWithWeight> points;

  for (int i = 0; i < sensor_width; i++)
  {
    for (int j = 0; j < sensor_height; j++)
    {
      if (transition_sum_map[i][j] > minimum_transitions_threshold)
      {
        PointWithWeight p;
        p.point = cv::Point(i, j);
        p.weight = (double) transition_sum_map[i][j];
        points.push_back(p);
      }
    }
  }

  return BoardDetection::findPattern(points);

//  std::list<cv::Point>::iterator point_it, cluster_it;
//  while (!points.empty())
//  {
//    std::list<cv::Point> current_cluster;
//    point_it = points.begin();
//    current_cluster.push_back(*point_it);
//    point_it = points.erase(point_it);
//
//    bool found_a_neighbor = true;
//    while (found_a_neighbor)
//    {
//      found_a_neighbor = false;
//      for (point_it = points.begin(); point_it != points.end(); ++point_it)
//      {
//        for (cluster_it = current_cluster.begin(); cluster_it != current_cluster.end(); ++cluster_it)
//        {
//          cv::Point dist = *point_it - *cluster_it;
//          if (dist.x >= -1 && dist.x <= 1 && dist.y >= -1 && dist.y <= 1)
//          {
//            current_cluster.push_back(cv::Point(*point_it));
//            point_it = points.erase(point_it);
//            found_a_neighbor = true;
//            break;
//          }
//        }
//      }
//    }
//
//    int cluster_mass = 0;
//    for (cluster_it = current_cluster.begin(); cluster_it != current_cluster.end(); ++cluster_it)
//      cluster_mass += transition_sum_map[cluster_it->x][cluster_it->y];
//    if (cluster_mass >= minimum_led_mass)
//      clusters.push_back(current_cluster);
//  }
//
//  std::vector<cv::Point2f> centers_good;
//  if (clusters.size() == dots * dots)
//  {
//    std::vector<cv::Point2f> centers;
//    for (int i = 0; i < clusters.size(); ++i)
//    {
//      cv::Point2f center(0, 0);
//      int mass = 0;
//      std::list<cv::Point>::iterator cluster_it;
//      for (cluster_it = clusters[i].begin(); cluster_it != clusters[i].end(); ++cluster_it)
//      {
//        center.x += cluster_it->x * transition_sum_map[cluster_it->x][cluster_it->y];
//        center.y += cluster_it->y * transition_sum_map[cluster_it->x][cluster_it->y];
//        mass += transition_sum_map[cluster_it->x][cluster_it->y];
//      }
//      center.x /= (double)mass;
//      center.y /= (double)mass;
//      centers.push_back(center);
//    }
//
//    CirclesGridClusterFinder grid(false);
//    grid.findGrid(centers, patternsize, centers_good);
//  }
//
//  return centers_good;
}

void DvsCalibration::calibrate()
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

void DvsCalibration::publishVisualizationImage(std::vector<cv::Point2f> pattern)
{
  cv_bridge::CvImage cv_image;
  cv_image.encoding = "bgr8";
  cv_image.image = cv::Mat(sensor_height, sensor_width, CV_8UC3);
  cv_image.image = cv::Scalar(255, 255, 255);
  for (int i = 0; i < sensor_width; i++)
  {
    for (int j = 0; j < sensor_height; j++)
    {
      int value = 255.0 - ((double)transition_sum_map[i][j]) / ((double)enough_transitions_threshold) * 255.0;
      cv_image.image.at<cv::Vec3b>(j, i) = cv::Vec3b(value, value, value);
    }
  }

  if (pattern.size() == dots * dots)
    cv::drawChessboardCorners(cv_image.image, cv::Size(dots, dots), cv::Mat(pattern), true);

  visualizationPublisher.publish(cv_image.toImageMsg());
}

void DvsCalibration::publishPatternImage(cv::Mat image)
{
  cv_bridge::CvImage cv_image;
  cv_image.encoding = "mono8";
  cv_image.image = image.clone();
  patternPublisher.publish(cv_image.toImageMsg());
}

void DvsCalibration::resetIntrinsicCalibration()
{
  publishPatternImage(pattern.get_intrinsic_calibration_pattern());
  object_points.clear();
  image_points.clear();
  num_detections = 0;
}

bool DvsCalibration::startCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  calibration_running = true;
  calibrate();
  return true;
}

bool DvsCalibration::resetCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  calibration_running = false;
  resetIntrinsicCalibration();
  return true;
}

bool DvsCalibration::saveCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  return setCameraInfo();
}

bool DvsCalibration::setCameraInfo()
{
  sensor_msgs::SetCameraInfo srv;
  srv.request.camera_info = cameraInfo;
  return setCameraInfoClient.call(srv);
}

void DvsCalibration::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  gotCameraInfo = true;
  cameraInfo = *msg;
}

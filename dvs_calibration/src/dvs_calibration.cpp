#include "dvs_calibration/dvs_calibration.h"

DvsCalibration::DvsCalibration()
{
  dots = 4;
  mode = WINDOW_OUTLINE;

  for (int i = -45; i <= 45; i += 15)
  {
    for (int j = -45; j <= 45; j += 15)
    {
      double roll = i / 180.0 * M_PI;
      double pitch = j / 180.0 * M_PI;
      Eigen::Matrix3d R;
      R = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
      orientations.push_back(R);
    }
  }
  orientation_id = 0;

  instrinsic_calibration_running_ = false;

  startCalibrationService = nh.advertiseService("/start_calibration", &DvsCalibration::startCalibrationCallback, this);
  resetService = nh.advertiseService("/reset_calibration", &DvsCalibration::resetCallback, this);

  eventSubscriber = nh.subscribe("/dvs_events", 10, &DvsCalibration::eventsCallback, this);
  detectionPublisher = nh.advertise<std_msgs::Int32>("/pattern_detections", 1);
  cameraInfoPublisher = nh.advertise<sensor_msgs::CameraInfo>("/dvs_camera_info", 1);
  reprojectionErrorPublisher = nh.advertise<std_msgs::Float64>("/calibration_reprojection_error", 1);

  image_transport::ImageTransport it(nh);
  patternPublisher = it.advertise("dvs_calib/pattern", 1);
  visualizationPublisher = it.advertise("dvs_calib/visualization", 1);

  f = boost::bind(&DvsCalibration::dynamicReconfigureCallback, this, _1, _2);
  srv.setCallback(f);
}

void DvsCalibration::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  if (mode != INTRINSIC_CALIBRATION)
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
    ROS_ERROR("enough transitions: %i out of %i", centers.size(), dots * dots);
    if (centers.size() == dots * dots)
    {
      publishVisualizationImage(centers);

      if (instrinsic_calibration_running_)
      {
        if (orientation_id < orientations.size())
        {
          image_points.push_back(centers);

          std::vector<cv::Point3f> world_pattern;
          for (int i = 0; i < dots; i++)
          {
            for (int j = 0; j < dots; j++)
            {
              world_pattern.push_back(cv::Point3f(i * 0.05, j * 0.05, 0.0));
            }
          }

          object_points.push_back(world_pattern);
          publishPatternImage(pattern.get_intrinsic_calibration_pattern(orientations[orientation_id]));

          orientation_id++;

          // send status
          std_msgs::Int32 msg;
          msg.data = orientation_id * 100.0 / ((double)orientations.size());
          detectionPublisher.publish(msg);
        }

        if (orientation_id == orientations.size())
        {
          ROS_ERROR("Got them all!");
          calibrate();
        }
      }
    }

    reset_maps();
  }
  else
  {
    std::vector<cv::Point2f> centers;
    publishVisualizationImage(centers);
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

void DvsCalibration::dynamicReconfigureCallback(dvs_calibration::DvsCalibrationConfig& config, uint32_t level)
{
  dots = config.dots;
  pattern.set_dots(dots);
  pattern.border_size = config.border_size;
  pattern.blinking = config.blinking;

  switch (config.pattern)
  {
    case 0:
      mode = WINDOW_OUTLINE;
      publishPatternImage(pattern.get_window_outline_pattern());
      break;
    case 1:
      mode = FOCUS_ADJUSTMENT;
      publishPatternImage(pattern.get_focus_adjustment_pattern());
      break;
    case 2:
      mode = INTRINSIC_CALIBRATION;
      publishPatternImage(pattern.get_intrinsic_calibration_pattern());
      break;
  }
}

std::vector<cv::Point2f> DvsCalibration::findPattern()
{
  cv::Size patternsize(dots, dots); //number of centers
  std::vector<cv::Point2f> centers; //this will be filled by the detected centers
  std::vector<cv::Point2f> centers_tmp; //this will be filled by the detected centers
  std::vector<int> center_count;

  std::list<cv::Point> points;
  std::vector< std::list<cv::Point> > clusters;

  for (int i = 0; i < sensor_width; i++)
  {
    for (int j = 0; j < sensor_height; j++)
    {
      if (transition_sum_map[i][j] > minimum_transitions_threshold)
      {
        points.push_back(cv::Point(i, j));
      }
    }
  }

  ROS_ERROR("got points: %i", points.size());

  std::list<cv::Point>::iterator point_it, cluster_it;
  while (!points.empty())
  {
//    ROS_ERROR("start while");
    std::list<cv::Point> current_cluster;
    point_it = points.begin();
    current_cluster.push_back(*point_it);
    point_it = points.erase(point_it);

    bool found_a_neighbor = true;
    while (found_a_neighbor)
    {
      found_a_neighbor = false;
      for (point_it=points.begin(); point_it!=points.end(); ++point_it)
      {
//        ROS_ERROR(" point iteration");
        for (cluster_it=current_cluster.begin(); cluster_it!=current_cluster.end(); ++cluster_it)
        {
//          ROS_ERROR("  cluster iteration");
          cv::Point dist = *point_it - *cluster_it;
          if (dist.x >= -1 && dist.x <= 1 && dist.y >= -1 && dist.y <= 1)
          {
            current_cluster.push_back(cv::Point(*point_it));
            point_it = points.erase(point_it);
            found_a_neighbor = true;
//            ROS_ERROR("   --> found_a_neighbor");
            break;
          }
        }
      }
    }

    if (current_cluster.size() >= 3)
    clusters.push_back(current_cluster);
  }

  ROS_ERROR("got clusters: %i", clusters.size());

  std::vector<cv::Point2f> centers_good;
  if (clusters.size() == dots*dots)
  {
    std::vector<cv::Point2f> centers;
    for (int i=0; i<clusters.size(); ++i)
    {
      cv::Point2f center(0, 0);
      std::list<cv::Point>::iterator cluster_it;
      for (cluster_it=clusters[i].begin(); cluster_it!=clusters[i].end(); ++cluster_it)
      {
        center.x += cluster_it->x;
        center.y += cluster_it->y;
      }
      center.x /= (double) clusters[i].size();
      center.y /= (double) clusters[i].size();
      centers.push_back(center);
    }

    CirclesGridClusterFinder grid(false);
    grid.findGrid(centers, patternsize, centers_good);
  }

  return centers_good;
}

void DvsCalibration::calibrate()
{
  instrinsic_calibration_running_ = false;
  cv::Mat cameraMatrix, distCoeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  double reproj_error = cv::calibrateCamera(object_points, image_points, cv::Size(sensor_width, sensor_height),
                                            cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_FIX_K3);

  sensor_msgs::CameraInfo msg;
  msg.height = sensor_height;
  msg.width = sensor_width;
  msg.distortion_model = "plumb_bob";

  for (int i = 0; i < distCoeffs.cols; i++)
    msg.D.push_back(distCoeffs.at<double>(i));
  for (int i = 0; i < 9; i++)
    msg.K[i] = cameraMatrix.at<double>(i);

  cameraInfoPublisher.publish(msg);

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

  if (pattern.size() == dots*dots)
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
  orientation_id = 0;
  publishPatternImage(pattern.get_intrinsic_calibration_pattern());
  object_points.clear();
  image_points.clear();
}

bool DvsCalibration::startCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  instrinsic_calibration_running_ = true;
  return true;
}

bool DvsCalibration::resetCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  instrinsic_calibration_running_ = false;
  resetIntrinsicCalibration();
  return true;
}

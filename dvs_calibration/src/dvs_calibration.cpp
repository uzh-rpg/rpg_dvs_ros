#include "dvs_calibration/dvs_calibration.h"

DvsCalibration::DvsCalibration()
{
  startCalibrationService = nh.advertiseService("dvs_calibration/start", &DvsCalibration::startCalibrationCallback, this);
  saveCalibrationService = nh.advertiseService("dvs_calibration/save", &DvsCalibration::saveCalibrationCallback, this);
  resetCalibrationService = nh.advertiseService("dvs_calibration/reset", &DvsCalibration::resetCalibrationCallback, this);

  numDetectionsPublisher = nh.advertise<std_msgs::Int32>("dvs_calibration/pattern_detections", 1);

  calibration_running = false;
  num_detections = 0;


  for (int i = 0; i < dots; i++)
  {
    for (int j = 0; j < dots; j++)
    {
      world_pattern.push_back(cv::Point3f(i * dot_distance , j * dot_distance , 0.0));
    }
  }
}

void DvsCalibration::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg, int camera_id)
{
  if (calibration_running)
    return;

  transition_maps[camera_id].update(msg);
  if (transition_maps[camera_id].max() > enough_transitions_threshold) {
    transition_maps[camera_id].find_pattern();
    if (transition_maps[camera_id].has_pattern()) {
      ROS_INFO("Has pattern...");
      add_pattern(camera_id);

      std_msgs::Int32 msg;
      msg.data = num_detections;
      numDetectionsPublisher.publish(msg);

      transition_maps[camera_id].reset_maps();
    }
  }

  update_visualization(camera_id);

  // reset if nothing is found after certain amount of time
  if (ros::Time::now() - transition_maps[camera_id].get_last_reset_time() > ros::Duration(pattern_search_timeout)) {
    ROS_INFO("calling reset because of time...");
    transition_maps[camera_id].reset_maps();
  }
}

bool DvsCalibration::resetCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("reset call");
  resetCalibration();
  return true;
}

bool DvsCalibration::startCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("start calib call");
  startCalibration();
  return true;
}

bool DvsCalibration::saveCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("save calib call");
  saveCalibration();
  return true;
}



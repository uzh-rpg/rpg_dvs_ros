// This file is part of DVS-ROS - the RPG DVS ROS Package

#include "dvs_calibration/dvs_calibration.h"

namespace dvs_calibration {

DvsCalibration::DvsCalibration()
{
  start_calibration_service_ = nh_.advertiseService("dvs_calibration/start", &DvsCalibration::startCalibrationCallback, this);
  save_calibration_service_ = nh_.advertiseService("dvs_calibration/save", &DvsCalibration::saveCalibrationCallback, this);
  reset_calibration_service_ = nh_.advertiseService("dvs_calibration/reset", &DvsCalibration::resetCalibrationCallback, this);

  num_detections_pub_ = nh_.advertise<std_msgs::Int32>("dvs_calibration/pattern_detections", 1);
  calibration_output_pub_ = nh_.advertise<std_msgs::String>("dvs_calibration/output", 1);

  calibration_running_ = false;
  num_detections_ = 0;

  // load parameters
  loadCalibrationParameters();

  for (int i = 0; i < params_.dots_h; i++)
  {
    for (int j = 0; j < params_.dots_w; j++)
    {
      world_pattern_.push_back(cv::Point3f(i * params_.dot_distance , j * params_.dot_distance , 0.0));
    }
  }
}

void DvsCalibration::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg, int camera_id)
{
  if (calibration_running_)
    return;

  sensor_width_ = msg->width;
  sensor_height_ = msg->height;

  transition_maps_[camera_id].update(msg);
  if (transition_maps_[camera_id].max() > params_.enough_transitions_threshold) {
    transition_maps_[camera_id].find_pattern();
    if (transition_maps_[camera_id].has_pattern()) {
      ROS_DEBUG("Found pattern.");
      addPattern(camera_id);

      std_msgs::Int32 msg;
      msg.data = num_detections_;
      num_detections_pub_.publish(msg);

      updateVisualization(camera_id);
    }

    transition_maps_[camera_id].reset_maps();
  }
  else {
    updateVisualization(camera_id);
  }

  // reset if nothing is found after certain amount of time
  if (ros::Time::now() - transition_maps_[camera_id].get_last_reset_time() > ros::Duration(params_.pattern_search_timeout)) {
    ROS_DEBUG("Reset maps because of time.");
    transition_maps_[camera_id].reset_maps();
  }
}

void DvsCalibration::loadCalibrationParameters()
{
  ros::NodeHandle nh_private("~");
  nh_private.param<int>("blinking_time_us", params_.blinking_time_us, 1000);
  nh_private.param<int>("blinking_time_tolerance_us", params_.blinking_time_tolerance_us, 500);
  nh_private.param<int>("enough_transitions_threshold", params_.enough_transitions_threshold, 200);
  nh_private.param<int>("minimum_transitions_threshold", params_.minimum_transitions_threshold, 10);
  nh_private.param<int>("minimum_led_mass", params_.minimum_led_mass, 50);
  nh_private.param<int>("dots_w", params_.dots_w, 5);
  nh_private.param<int>("dots_h", params_.dots_h, 5);
  nh_private.param<double>("dot_distance", params_.dot_distance, 0.05);
  nh_private.param<double>("pattern_search_timeout", params_.pattern_search_timeout, 2.0);  
}

bool DvsCalibration::resetCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Reset call");
  resetCalibration();
  return true;
}

bool DvsCalibration::startCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Start calibration call");
  startCalibration();
  return true;
}

bool DvsCalibration::saveCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Save calibration call");
  saveCalibration();
  return true;
}

} // namespace


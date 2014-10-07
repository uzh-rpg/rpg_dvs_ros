#include "dvs_calibration/dvs_calibration.h"

DvsCalibration::DvsCalibration()
{
  startCalibrationService = nh.advertiseService("dvs_calibration/start", &DvsCalibration::startCalibrationCallback, this);
  saveCalibrationService = nh.advertiseService("dvs_calibration/save", &DvsCalibration::saveCalibrationCallback, this);
  resetCalibrationService = nh.advertiseService("dvs_calibration/reset", &DvsCalibration::resetCalibrationCallback, this);

  calibration_running = false;
}

void DvsCalibration::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg, int camera_id)
{
  ROS_INFO("Got events from camera with id %d", camera_id);

   transition_maps[camera_id].update(msg);
   if (transition_maps[camera_id].max() > enough_transitions_threshold) {
     if (transition_maps[camera_id].has_pattern()) {
       ROS_INFO("Has pattern...");
//       check_stereo_pattern();
     }
   }
}

bool DvsCalibration::resetCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  resetCalibration();
  return true;
}

bool DvsCalibration::startCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  startCalibration();
  return true;
}

bool DvsCalibration::saveCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  saveCalibration();
  return true;
}



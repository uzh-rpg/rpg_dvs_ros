#ifndef MONO_DVS_CALIBRATION_H
#define MONO_DVS_CALIBRATION_H

#include <list>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "dvs_calibration/dvs_calibration.h"


namespace dvs_calibration {

class MonoDvsCalibration : public DvsCalibration
{
public:
  MonoDvsCalibration();
  virtual ~MonoDvsCalibration(){};

private:
  void resetCalibration();
  void startCalibration();
  void saveCalibration();

  void add_pattern(int id);
  void update_visualization(int id);

  // services
  sensor_msgs::CameraInfo cameraInfo;
  bool setCameraInfo();

  void calibrate();
  void publishVisualizationImage(cv::Mat image);

  // calibration
  std::vector< std::vector<cv::Point3f> > object_points;
  std::vector< std::vector<cv::Point2f> > image_points;

  // ROS interface
  ros::Subscriber eventSubscriber;
  image_transport::Publisher visualizationPublisher;
  image_transport::Publisher patternPublisher;
  ros::ServiceClient setCameraInfoClient;
};

} // namespace

#endif // MONO_DVS_CALIBRATION_H

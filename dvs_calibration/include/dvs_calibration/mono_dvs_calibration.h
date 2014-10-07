#ifndef MONO_DVS_CALIBRATION_H
#define MONO_DVS_CALIBRATION_H

#include "ros/ros.h"

#include <Eigen/Geometry>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include "dvs_calibration/pattern.h"
#include "dvs_calibration/circlesgrid.hpp"
#include "dvs_calibration/board_detection.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/PoseStamped.h>

#include <list>

#include "dvs_calibration/dvs_calibration.h"

class MonoDvsCalibration : public DvsCalibration
{
public:
  MonoDvsCalibration();
  virtual ~MonoDvsCalibration() {};

private:
  // callbacks
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void resetCalibration();
  void startCalibration();
  void saveCalibration();

  // services
  sensor_msgs::CameraInfo cameraInfo;
  bool setCameraInfo();

  void calibrate();
  void publishVisualizationImage(std::vector<cv::Point2f> pattern);
  void publishPatternImage(cv::Mat image);

  // calibration
  std::vector< std::vector<cv::Point3f> > object_points;
  std::vector< std::vector<cv::Point2f> > image_points;

  // ROS interface
  ros::Subscriber eventSubscriber;
  ros::Subscriber cameraInfoSubscriber;
  ros::Publisher detectionPublisher;
  ros::Publisher cameraInfoPublisher;
  ros::Publisher reprojectionErrorPublisher;
  ros::Publisher cameraPosePublisher;
  image_transport::Publisher visualizationPublisher;
  image_transport::Publisher patternPublisher;
  ros::ServiceClient setCameraInfoClient;

  // for pose publishing
  bool gotCameraInfo;
  int num_detections;

};

#endif // MONO_DVS_CALIBRATION_H

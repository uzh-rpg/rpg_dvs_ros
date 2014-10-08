#ifndef STEREO_DVS_CALIBRATION_H
#define STEREO_DVS_CALIBRATION_H

#include "ros/ros.h"

#include <Eigen/Geometry>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
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

class StereoDvsCalibration : public DvsCalibration
{
public:
  StereoDvsCalibration();
  virtual ~StereoDvsCalibration() {};

private:
  // callbacks
  void cameraInfoLeftCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void cameraInfoRightCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void resetCalibration();
  void startCalibration();
  void saveCalibration();

  void add_pattern(int id);
  void update_visualization(int id);

  // services
  sensor_msgs::CameraInfo cameraInfoLeft, cameraInfoRight;
  bool setCameraInfo();

  void calibrate();
  void publishVisualizationImage(cv::Mat image, int id);

  // calibration
  std::vector< std::vector<cv::Point3f> > object_points;
  std::vector< std::vector<cv::Point2f> > image_points_left, image_points_right;

  // ROS interface
  ros::Subscriber eventLeftSubscriber, eventRightSubscriber;
  ros::Subscriber cameraInfoLeftSubscriber, cameraInfoRightSubscriber;
  ros::Publisher cameraInfoPublisher;
  ros::Publisher reprojectionErrorPublisher;
  ros::Publisher cameraPosePublisher;
  image_transport::Publisher visualizationLeftPublisher, visualizationRightPublisher;
  ros::ServiceClient setCameraInfoLeftClient, setCameraInfoRightClient;

  // buffer to wait for other camera
  void add_stereo_pattern(std::vector<cv::Point2f> left, std::vector<cv::Point2f> right);
  std::vector<cv::Point2f> image_point_buffer;
  bool has_left_buffer, has_right_buffer;
  ros::Time buffer_time;

  // for pose publishing
  bool gotCameraInfoLeft, gotCameraInfoRight;
};

#endif // STEREO_DVS_CALIBRATION_H

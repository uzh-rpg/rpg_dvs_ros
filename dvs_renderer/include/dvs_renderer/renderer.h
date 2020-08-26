// This file is part of DVS-ROS - the RPG DVS ROS Package

#ifndef DVS_RENDERER_H_
#define DVS_RENDERER_H_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "image_tracking.h"

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

namespace dvs_renderer
{

class Renderer {
public:
  Renderer(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  virtual ~Renderer();

private:
  ros::NodeHandle nh_;

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

  void publishStats();

  bool got_camera_info_;
  bool color_image_;
  cv::Mat camera_matrix_, dist_coeffs_;

  ros::Subscriber event_sub_;
  ros::Subscriber camera_info_sub_;

  image_transport::Publisher image_pub_;
  image_transport::Publisher undistorted_image_pub_;

  image_transport::Subscriber image_sub_;
  cv::Mat last_image_;
  bool used_last_image_;

  struct EventStats {
    ros::Publisher events_mean_[2]; /**< event stats output */
    int events_counter_[2]; /**< event counters for on/off events */
    double events_mean_lasttime_;
    double dt;
  };
  EventStats event_stats_[2]; /**< event statistics for 1 and 5 sec */

  enum DisplayMethod
  {
    GRAYSCALE, RED_BLUE
  } display_method_;

  ImageTracking image_tracking_;
};

} // namespace

#endif // DVS_RENDERER_H_

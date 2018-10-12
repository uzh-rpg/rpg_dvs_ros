// This file is part of DVS-ROS - the RPG DVS ROS Package

#pragma once

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

namespace dvs_renderer
{

/**
 * @class ImageTracking
 * Track frames over 1 sec, then project all events during this time into
 * the latest frame & output the accumulated events per pixel.
 * This requires the scene to be planar, and for best results, the camera motion
 * is perpendicular to the plane's surface normal.
 */
class ImageTracking {
public:
  ImageTracking(ros::NodeHandle & nh);
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
private:

  void render();
  void reset();

  struct ImgData {
    cv::Mat img;
    ros::Time t;
    std::vector<cv::Point2f> points;
    cv::Mat homography; //homography from previous image to this
    cv::Mat homography_accumulated; //homography from previous image to the second last
    cv::Point2f translation; //mean translation, previous to this
  };
  std::vector<ImgData> images_;
  std::vector<dvs_msgs::Event> events_;
  double start_time_;
  cv::Mat edges_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher image_pub_events_edges_;
};


} // namespace

// This file is part of DVS-ROS - the RPG DVS ROS Package

#include "dvs_renderer/renderer.h"
#include <std_msgs/Float32.h>

namespace dvs_renderer {

Renderer::Renderer(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh),
    image_tracking_(nh)
{
  got_camera_info_ = false;

  // get parameters of display method
  std::string display_method_str;
  nh_private.param<std::string>("display_method", display_method_str, "");
  display_method_ = (display_method_str == std::string("grayscale")) ? GRAYSCALE : RED_BLUE;
  nh_private.param<bool>("color", color_image_, false);
  used_last_image_ = false;

  // setup subscribers and publishers
  event_sub_ = nh_.subscribe("events", 1, &Renderer::eventsCallback, this);
  camera_info_sub_ = nh_.subscribe("camera_info", 1, &Renderer::cameraInfoCallback, this);

  image_transport::ImageTransport it_(nh_);
  image_sub_ = it_.subscribe("image", 1, &Renderer::imageCallback, this);
  image_pub_ = it_.advertise("dvs_rendering", 1);
  undistorted_image_pub_ = it_.advertise("dvs_undistorted", 1);

  for (int i = 0; i < 2; ++i)
    for (int k = 0; k < 2; ++k)
      event_stats_[i].events_counter_[k] = 0;
  event_stats_[0].dt = 1;
  event_stats_[0].events_mean_lasttime_ = 0;
  event_stats_[0].events_mean_[0] = nh_.advertise<std_msgs::Float32>("events_on_mean_1", 1);
  event_stats_[0].events_mean_[1] = nh_.advertise<std_msgs::Float32>("events_off_mean_1", 1);
  event_stats_[1].dt = 5;
  event_stats_[1].events_mean_lasttime_ = 0;
  event_stats_[1].events_mean_[0] = nh_.advertise<std_msgs::Float32>("events_on_mean_5", 1);
  event_stats_[1].events_mean_[1] = nh_.advertise<std_msgs::Float32>("events_off_mean_5", 1);
}

Renderer::~Renderer()
{
  image_pub_.shutdown();
  undistorted_image_pub_.shutdown();
}

void Renderer::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  got_camera_info_ = true;

  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      camera_matrix_.at<double>(cv::Point(i, j)) = msg->K[i+j*3];

  dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
  for (int i = 0; i < msg->D.size(); i++)
    dist_coeffs_.at<double>(i) = msg->D[i];
}

void Renderer::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  image_tracking_.imageCallback(msg);

  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // convert to BGR image
  if (msg->encoding == "rgb8") cv::cvtColor(cv_ptr->image, last_image_, CV_RGB2BGR);
  if (msg->encoding == "mono8")
  {
    if (color_image_)
    {
      cv::cvtColor(cv_ptr->image, last_image_, CV_BayerBG2BGR);
    }
    else
    {
      cv::cvtColor(cv_ptr->image, last_image_, CV_GRAY2BGR);
    }
  }

  if (!used_last_image_)
  {
    cv_bridge::CvImage cv_image;
    last_image_.copyTo(cv_image.image);
    cv_image.encoding = "bgr8";
    std::cout << "publish image from callback" << std::endl;
    image_pub_.publish(cv_image.toImageMsg());
  }
  used_last_image_ = false;
}

void Renderer::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  for (int i = 0; i < msg->events.size(); ++i)
  {
    ++event_stats_[0].events_counter_[msg->events[i].polarity];
    ++event_stats_[1].events_counter_[msg->events[i].polarity];
  }

  publishStats();
  image_tracking_.eventsCallback(msg);

  // only create image if at least one subscriber
  if (image_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage cv_image;
    if (msg->events.size() > 0)
    {
      cv_image.header.stamp = msg->events[msg->events.size()/2].ts;
    }

    if (display_method_ == RED_BLUE)
    {
      cv_image.encoding = "bgr8";

      if (last_image_.rows == msg->height && last_image_.cols == msg->width)
      {
        last_image_.copyTo(cv_image.image);
        used_last_image_ = true;
      }
      else
      {
        cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC3);
        cv_image.image = cv::Scalar(0,0,0);
      }

      for (int i = 0; i < msg->events.size(); ++i)
      {
        const int x = msg->events[i].x;
        const int y = msg->events[i].y;

        cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = (
            msg->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
      }
    }
    else
    {
      cv_image.encoding = "mono8";
      if (last_image_.rows == msg->height && last_image_.cols == msg->width)
      {
        cv::cvtColor(last_image_, cv_image.image, CV_BGR2GRAY);
        used_last_image_ = true;
      }
      else
      {
        cv_image.image = cv::Mat(msg->height, msg->width, CV_8U);
        cv_image.image = cv::Scalar(128);
      }

      cv::Mat on_events = cv::Mat(msg->height, msg->width, CV_8U);
      on_events = cv::Scalar(0);

      cv::Mat off_events = cv::Mat(msg->height, msg->width, CV_8U);
      off_events = cv::Scalar(0);

      // count events per pixels with polarity
      for (int i = 0; i < msg->events.size(); ++i)
      {
        const int x = msg->events[i].x;
        const int y = msg->events[i].y;

        if (msg->events[i].polarity == 1)
          on_events.at<uint8_t>(cv::Point(x, y))++;
        else
          off_events.at<uint8_t>(cv::Point(x, y))++;
      }

        // scale image
      cv::normalize(on_events, on_events, 0, 128, cv::NORM_MINMAX, CV_8UC1);
      cv::normalize(off_events, off_events, 0, 127, cv::NORM_MINMAX, CV_8UC1);

      cv_image.image += on_events;
      cv_image.image -= off_events;
    }

    image_pub_.publish(cv_image.toImageMsg());

    if (got_camera_info_ && undistorted_image_pub_.getNumSubscribers() > 0)
    {
      cv_bridge::CvImage cv_image2;
      cv_image2.encoding = cv_image.encoding;
      cv::undistort(cv_image.image, cv_image2.image, camera_matrix_, dist_coeffs_);
      undistorted_image_pub_.publish(cv_image2.toImageMsg());
    }
  }
}

void Renderer::publishStats()
{
  std_msgs::Float32 msg;
  ros::Time now = ros::Time::now();
  for (int i = 0; i < 2; ++i)
  {
    if (event_stats_[i].events_mean_lasttime_ + event_stats_[i].dt <= now.toSec()) {
      event_stats_[i].events_mean_lasttime_ = now.toSec();
      for (int k = 0; k < 2; ++k)
      {
        msg.data = (float)event_stats_[i].events_counter_[k] / event_stats_[i].dt;
        event_stats_[i].events_mean_[k].publish(msg);
        event_stats_[i].events_counter_[k] = 0;
      }
    }
  }
}

} // namespace

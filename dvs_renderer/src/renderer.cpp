// This file is part of DVS-ROS - the RPG DVS ROS Package

#include "dvs_renderer/renderer.h"
#include <std_msgs/Float32.h>
#include <opencv2/opencv.hpp>

namespace dvs_renderer {

Renderer::Renderer(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh),
    image_tracking_(nh)
{
  got_camera_info_ = false;

  static int buffer = 100;

  // get parameters of display method
  std::string display_method_str;
  nh_private.param<std::string>("display_method", display_method_str, "");
  display_method_ = (display_method_str == std::string("grayscale")) ? GRAYSCALE : RED_BLUE;

  // setup subscribers and publishers
  event_sub_ = nh_.subscribe("events", buffer, &Renderer::eventsCallback, this);
  camera_info_sub_ = nh_.subscribe("camera_info", buffer, &Renderer::cameraInfoCallback, this);

  image_transport::ImageTransport it_(nh_);
  image_sub_ = it_.subscribe("image", buffer, &Renderer::imageCallback, this);
  image_pub_ = it_.advertise("dvs_rendering", buffer);
  undistorted_image_pub_ = it_.advertise("dvs_undistorted", buffer);
  image_raw_pub_ = it_.advertise("dvs_image_raw", buffer);
  event_image_pub_ = it_.advertise("event_image", buffer);

  used_last_image_ = false;
  received_image_ = false;

}

Renderer::~Renderer()
{
  image_pub_.shutdown();
  undistorted_image_pub_.shutdown();
  image_raw_pub_.shutdown();
  event_image_pub_.shutdown();
}

void Renderer::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{

}

void Renderer::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  image_tracking_.imageCallback(msg);

  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // convert from grayscale to color image
  cv::cvtColor(cv_ptr->image, last_image_, CV_GRAY2BGR);
  cv_ptr->image.copyTo(image_raw_);

  if (!used_last_image_ && false)
  {
    cv_bridge::CvImage cv_image;
    last_image_.copyTo(cv_image.image);
    cv_image.encoding = "bgr8";
    std::cout << "publish image from callback" << std::endl;
    image_pub_.publish(cv_image.toImageMsg());
  }
  used_last_image_ = false;

  received_image_ = true;
}

void Renderer::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  static int counter = 0;
  if (received_image_)
	{
    cv_bridge::CvImage cv_image;
    cv_bridge::CvImage cv_image_raw;
    cv_bridge::CvImage cv_image_events;

    cv::Mat img_event;
    cv::Mat img_raw;

    if (msg->events.size() > 0)
    {
      cv_image.header.stamp = msg->events[msg->events.size()/2].ts;
      cv_image_raw.header.stamp = msg->events[msg->events.size()/2].ts;
      cv_image_events.header.stamp = msg->events[msg->events.size()/2].ts;

      cv_image.encoding = "bgr8";
      cv_image_raw.encoding = "mono8";
      cv_image_events.encoding = "bgr8";

      cv_image_raw.image = cv::Mat(msg->height, msg->width, CV_8UC1);
      cv_image_raw.image = cv::Scalar(255);

      cv_image_events.image = cv::Mat(msg->height, msg->width, CV_8UC3);
      cv_image_events.image = cv::Scalar(255, 255 ,255);

      img_event = cv::Mat(msg->height, msg->width, CV_8UC3);
      img_event = cv::Scalar(255, 255 ,255);

      for (int i = 0; i < msg->events.size(); ++i)
      {
        const int x = msg->events[i].x;
        const int y = msg->events[i].y;

        cv_image_events.image.at<cv::Vec3b>(cv::Point(x, y)) = (
            msg->events[i].polarity == false ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));

        img_event.at<cv::Vec3b>(cv::Point(x, y)) = (
            msg->events[i].polarity == false ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
      }

    image_raw_.copyTo(img_raw);
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(0);

    char name [200];

    sprintf(name, "/home/cedric/Documents/phd/data/image_raw/image%04d.png", counter);

    cv::imwrite(name, img_raw, compression_params);

    sprintf(name, "/home/cedric/Documents/phd/data/events_red_blue/image%04d.png", counter);

    cv::imwrite(name, img_event, compression_params);

//    image_raw_.copyTo(cv_image_raw.image);
//    image_raw_pub_.publish(cv_image_raw.toImageMsg());
//    event_image_pub_.publish(cv_image_events.toImageMsg());

    counter++;

    }
	}
}
} // namespace

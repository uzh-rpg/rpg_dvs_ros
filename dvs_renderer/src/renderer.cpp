#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

image_transport::Publisher image_pub_;

enum DisplayMethod {
  GRAYSCALE,
  RED_BLUE
} display_method;

void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // only create image if at least one subscriber
  if (image_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage cv_image;

    if (display_method == RED_BLUE)
    {
      cv_image.encoding = "bgr8";
      cv_image.image = cv::Mat(128, 128, CV_8UC3);
      cv_image.image = cv::Scalar(128, 128, 128);

      for (int i = 0; i < msg->events.size(); ++i)
      {
        int x = msg->events[i].x;
        int y = msg->events[i].y;

        cv_image.image.at<cv::Vec3b>(y, x) = (
            msg->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
      }
    }
    else
    {
      cv_image.encoding = "mono8";
      cv_image.image = cv::Mat(128, 128, CV_8U);
      cv_image.image = cv::Scalar(128);

      cv::Mat on_events = cv::Mat(128, 128, CV_8U);
      on_events = cv::Scalar(0);

      cv::Mat off_events = cv::Mat(128, 128, CV_8U);
      off_events = cv::Scalar(0);

      // count events per pixels with polarity
      for (int i = 0; i < msg->events.size(); ++i)
      {
        int x = msg->events[i].x;
        int y = msg->events[i].y;

        if (msg->events[i].polarity == 1)
          on_events.at<uint8_t>(y, x)++;
        else
          off_events.at<uint8_t>(y, x)++;
      }

      // scale image
      cv::normalize(on_events, on_events, 0, 128, cv::NORM_MINMAX, CV_8UC1);
      cv::normalize(off_events, off_events, 0, 127, cv::NORM_MINMAX, CV_8UC1);

      cv_image.image += on_events;
      cv_image.image -= off_events;
    }

    image_pub_.publish(cv_image.toImageMsg());
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dvs_renderer");

  ros::NodeHandle nh;

  // get parameters of display method
  ros::NodeHandle nh_private("~");
  std::string display_method_str;
  nh_private.getParam("display_method", display_method_str);

  if (display_method_str == std::string("grayscale"))
    display_method = GRAYSCALE;
  else
    display_method = RED_BLUE;

  // setup subscribers and publishers
  ros::Subscriber sub = nh.subscribe("dvs_events", 1, eventsCallback);

  image_transport::ImageTransport it_(nh);
  image_pub_ = it_.advertise("dvs_rendering", 1);

  ros::spin();

  return 0;
}

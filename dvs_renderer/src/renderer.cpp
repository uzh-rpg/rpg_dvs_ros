#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

image_transport::Publisher image_pub_;

void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg) {
  cv_bridge::CvImage cv_image;

  cv_image.encoding = "bgr8";
  cv_image.image = cv::Mat(128, 128, CV_8UC3);
  cv_image.image = cv::Scalar(128, 128, 128);

  for (int i=0; i<msg->events.size(); ++i) {
    int x = msg->events[i].x;
    int y = msg->events[i].y;

    if (x < 0 || x > 127 || y < 0 || y > 127) {
      ROS_WARN("Pixel location out of range!");
    }

    cv_image.image.at<cv::Vec3b>(y,x) = (msg->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
  }

  image_pub_.publish(cv_image.toImageMsg());
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dvs_renderer");

  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("dvs_events", 1, eventsCallback);

  image_transport::ImageTransport it_(nh);
  image_pub_ = it_.advertise("dvs_rendering", 1);

  ros::spin();

  return 0;
}

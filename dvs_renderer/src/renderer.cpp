#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

image_transport::Publisher image_pub_;
image_transport::Publisher undistorted_image_pub_;

enum DisplayMethod
{
  GRAYSCALE, RED_BLUE
} display_method;

bool got_camera_info = false;
cv::Mat cameraMatrix, distCoeffs;

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  got_camera_info = true;

  cameraMatrix = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      cameraMatrix.at<double>(j, i) = msg->K[i+j*3];

  distCoeffs = cv::Mat(msg->D.size(), 1, CV_64F);
  for (int i = 0; i < msg->D.size(); i++)
    distCoeffs.at<double>(i) = msg->D[i];
}

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
          on_events.at<uint8_t>(y, x)++;else
          off_events.at<uint8_t>(y, x)++;}

        // scale image
      cv::normalize(on_events, on_events, 0, 128, cv::NORM_MINMAX, CV_8UC1);
      cv::normalize(off_events, off_events, 0, 127, cv::NORM_MINMAX, CV_8UC1);

      cv_image.image += on_events;
      cv_image.image -= off_events;
    }

    image_pub_.publish(cv_image.toImageMsg());

    if (got_camera_info && undistorted_image_pub_.getNumSubscribers() > 0)
    {
      cv_bridge::CvImage cv_image2;
      cv_image2.encoding = cv_image.encoding;
      cv::undistort(cv_image.image, cv_image2.image, cameraMatrix, distCoeffs);
      undistorted_image_pub_.publish(cv_image2.toImageMsg());
    }
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dvs_renderer");

  ros::NodeHandle nh;

  // get parameters of display method
  std::string display_method_str;
  ros::param::get("~display_method", display_method_str);

  if (display_method_str == std::string("grayscale"))
    display_method = GRAYSCALE;
  else
    display_method = RED_BLUE;

  // setup subscribers and publishers
  ros::Subscriber sub = nh.subscribe("events", 1, eventsCallback);
  ros::Subscriber sub2 = nh.subscribe("camera_info", 1, cameraInfoCallback);

  image_transport::ImageTransport it_(nh);
  image_pub_ = it_.advertise("dvs_rendering", 1);
  undistorted_image_pub_ = it_.advertise("dvs_undistorted", 1);

  ros::spin();

  image_pub_.shutdown();
  undistorted_image_pub_.shutdown();

  return 0;
}

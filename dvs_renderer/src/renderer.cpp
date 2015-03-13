// This file is part of DVS-ROS - the RPG DVS ROS Package
//
// DVS-ROS is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// DVS-ROS is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with DVS-ROS.  If not, see <http://www.gnu.org/licenses/>.

#include "dvs_renderer/renderer.h"

namespace dvs_renderer {

Renderer::Renderer(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh)
{
  got_camera_info_ = false;

  // get parameters of display method
  std::string display_method_str;
  nh_private.param<std::string>("display_method", display_method_str, "");
  display_method_ = (display_method_str == std::string("grayscale")) ? GRAYSCALE : RED_BLUE;

  // setup subscribers and publishers
  event_sub_ = nh_.subscribe("events", 1, &Renderer::eventsCallback, this);
  camera_info_sub_ = nh_.subscribe("camera_info", 1, &Renderer::cameraInfoCallback, this);

  image_transport::ImageTransport it_(nh_);
  image_pub_ = it_.advertise("dvs_rendering", 1);
  undistorted_image_pub_ = it_.advertise("dvs_undistorted", 1);
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

void Renderer::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // only create image if at least one subscriber
  if (image_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage cv_image;

    if (display_method_ == RED_BLUE)
    {
      cv_image.encoding = "bgr8";
      cv_image.image = cv::Mat(128, 128, CV_8UC3);
      cv_image.image = cv::Scalar(128, 128, 128);

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
      cv_image.image = cv::Mat(128, 128, CV_8U);
      cv_image.image = cv::Scalar(128);

      cv::Mat on_events = cv::Mat(128, 128, CV_8U);
      on_events = cv::Scalar(0);

      cv::Mat off_events = cv::Mat(128, 128, CV_8U);
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

} // namespace

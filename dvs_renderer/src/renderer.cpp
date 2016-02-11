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
#include <std_msgs/Float32.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace dvs_renderer {

Renderer::Renderer(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh),
    image_tracking_(nh)
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
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // convert from grayscale to color image
  cv::cvtColor(cv_ptr->image, last_image_, CV_GRAY2BGR);

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
        cv_image.image = cv::Scalar(128, 128, 128);
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
      cv_image.image = cv::Mat(msg->height, msg->width, CV_8U);
      cv_image.image = cv::Scalar(128);

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


ImageTracking::ImageTracking(ros::NodeHandle & nh)
{
  start_time_ = ros::Time::now().toSec();
  image_transport::ImageTransport it_(nh);
  image_pub_ = it_.advertise("dvs_accumulated_events", 1);
}
void ImageTracking::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  if (image_pub_.getNumSubscribers() == 0)
    return;

  for (const auto& event : msg->events)
    events_.push_back(event);
}

void ImageTracking::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  if (image_pub_.getNumSubscribers() == 0)
    return;

  ImgData img_data;
  cv_bridge::CvImagePtr cv_image_mono;
  cv_image_mono = cv_bridge::toCvCopy(msg, "mono8");
  img_data.img = cv_image_mono->image;
  img_data.t = msg->header.stamp;
  images_.push_back(img_data);
  ImgData& img = images_[images_.size() - 1];

  if (images_.size() == 1)
  {
    //find features to track...
    int maxCorners = 60;
    double qualityLevel = 0.01;
    double minDistance = 6;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;
    cv::Size size = img.img.size();
    cv::Mat mask = cv::Mat(size, CV_8UC1);
    mask.setTo(cv::Scalar::all(0));
    //restrict feature points to the center of the image
    float mask_size = 0.15;
    cv::Rect roi(size.width * mask_size, size.height * mask_size,
             size.width * (1.-mask_size*2), size.height * (1.-mask_size*2));
    mask(roi).setTo(cv::Scalar::all(255));

    cv::goodFeaturesToTrack(img.img, img.points, maxCorners, qualityLevel,
                            minDistance, mask, blockSize, useHarrisDetector, k);
  }
  else
  {
    ImgData& prev_img = images_[images_.size() - 2];
    //track features
    std::vector<uchar> status;
    std::vector<float> err;
    cv::Size win_size(20, 20);
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
    cv::calcOpticalFlowPyrLK(prev_img.img, img.img, prev_img.points, img.points,
                             status, err, win_size, 3, termcrit, 0, 0.001);
    //filter failed points
    cv::Point2f translation_sum(0.f, 0.f);
    int translation_sum_counter;
    for (int i = status.size()-1; i >= 0; --i)
    {
      if (status[i])
      {
        translation_sum += img.points[i] - prev_img.points[i];
        ++translation_sum_counter;
      }
      else
      {
        prev_img.points.erase(prev_img.points.begin()+i);
        img.points.erase(img.points.begin()+i);
      }
    }

    if (img.points.size() >= 4)
    {
      img.homography = cv::findHomography(prev_img.points, img.points, 0);
      img.translation = translation_sum * (1.f/translation_sum_counter);
    }
  }

  if (img.points.size() < 4)
  {
    reset();
  }
  else if (ros::Time::now().toSec() > start_time_ + 1.)
  {
    render();
    reset();
  }
}

void ImageTracking::render()
{
  if (images_.empty())
    return;

  cv::Size img_size = images_[0].img.size();
  cv::Mat output_img = cv::Mat::zeros(img_size, CV_16UC1);
  int num_events = 0;

  //accumulate homographies (for speedup)
  for (size_t i = 1; i < images_.size(); ++i)
  {
    images_[i].homography_accumulated = images_[i].homography;
    for (size_t j = i + 1; j < images_.size() - 1; ++j)
    {
      images_[i].homography_accumulated =
          images_[j].homography * images_[i].homography_accumulated;
    }
  }

  for (const auto& event : events_)
  {
    //ignore too old events
    if (event.ts < images_[0].t)
      continue;

    //find image: use timestamp in the middle of 2 images to decide
    size_t i = 1;
    while (i < images_.size() &&
        event.ts.toSec() > (images_[i-1].t.toSec()+images_[i].t.toSec())/2.) ++i;

    if (i >= images_.size() - 1)
      continue;

    //event belongs to images_[i-1]
    std::vector<cv::Point2f> v1, v2;
    v1.push_back(cv::Point2f(event.x, event.y));

    //in between images approximation: simple linear interpolation
    //TODO: something more accurate...
    int idx_offset = 0;
    if (event.ts < images_[i-1].t)
      idx_offset = -1;
    float s = (event.ts.toSec() - images_[i-1].t.toSec()) /
        (images_[i+idx_offset].t.toSec() - images_[i-1+idx_offset].t.toSec());
    v1[0] -= images_[i+idx_offset].translation * s;

    //now project into the second last image
    cv::perspectiveTransform(v1, v2, images_[i].homography_accumulated);

    //update event map
    cv::Point2f p = v2[0];
    int x = cvRound(p.x), y = cvRound(p.y);
    if (x >= 0 && x < img_size.width && y >= 0 && y < img_size.height)
    {
      ++output_img.at<uint16_t>(cv::Point(x, y));
      ++num_events;
    }

  }

  //require minimum number of events
  if (num_events < 10000) return;

  cv::normalize(output_img, output_img, 0, 65535, cv::NORM_MINMAX);

  cv_bridge::CvImage cv_image;
  cv_image.encoding = "bgr8";
  cv::Mat img_lower_depth;
  output_img.convertTo(img_lower_depth, CV_8U, 1./256);
  cv::cvtColor(img_lower_depth, cv_image.image, CV_GRAY2BGR);
  image_pub_.publish(cv_image.toImageMsg());
}

void ImageTracking::reset()
{
  images_.clear();
  events_.clear();
  start_time_ = ros::Time::now().toSec();
}

} // namespace

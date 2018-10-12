// This file is part of DVS-ROS - the RPG DVS ROS Package

#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "dvs_renderer/image_tracking.h"

namespace dvs_renderer {


ImageTracking::ImageTracking(ros::NodeHandle & nh)
{
  start_time_ = ros::Time::now().toSec();
  image_transport::ImageTransport it_(nh);
  image_pub_ = it_.advertise("dvs_accumulated_events", 1);
  image_pub_events_edges_ = it_.advertise("dvs_accumulated_events_edges", 1);
}
void ImageTracking::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  if (image_pub_.getNumSubscribers() == 0 && image_pub_events_edges_.getNumSubscribers() == 0)
    return;

  for (const auto& event : msg->events)
    events_.push_back(event);
}

void ImageTracking::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  if (image_pub_.getNumSubscribers() == 0 && image_pub_events_edges_.getNumSubscribers() == 0)
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
    cv::Point2f pos(event.x, event.y);
    v1.push_back(pos);

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
  //output_img.convertTo(img_lower_depth, CV_8U, 1./256 / nth_el_val);
  output_img.convertTo(img_lower_depth, CV_8U, 1./256);
  cv::cvtColor(img_lower_depth, cv_image.image, CV_GRAY2BGR);
  image_pub_.publish(cv_image.toImageMsg());


  int low_threshold = 30;
  double ratio = 3.;
  cv::blur(images_[images_.size()-2].img, edges_, cv::Size(3,3));
  cv::Canny(edges_, edges_, low_threshold, low_threshold*ratio, 3);
  cv::Mat channels[] = { edges_, img_lower_depth, img_lower_depth };
  cv::merge(channels, 3, cv_image.image);

  image_pub_events_edges_.publish(cv_image.toImageMsg());
}

void ImageTracking::reset()
{
  images_.clear();
  events_.clear();
  start_time_ = ros::Time::now().toSec();
}

} // namespace

#include "ros/ros.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "dvs_calibration/circlesgrid.hpp"
#include "dvs_calibration/pattern.h"

#include <iostream>
#include <vector>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

using namespace cv;
using namespace std;

int last_off_map[128][128];
int transition_sum_map[128][128];
const int max_diff = 10000;

void reset_maps()
{
  for (int i = 0; i < 128; i++)
  {
    for (int j = 0; j < 128; j++)
    {
      last_off_map[i][j] = 0;
      transition_sum_map[i][j] = 0;
    }
  }
}

void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  for (int i = 0; i < msg->events.size(); ++i)
  {
    if (msg->events[i].polarity == true)
    {
      last_off_map[msg->events[i].x][msg->events[i].y] = msg->events[i].time;
    }
    else
    {
      if (msg->events[i].time - last_off_map[msg->events[i].x][msg->events[i].y] < max_diff)
        transition_sum_map[msg->events[i].x][msg->events[i].y]++;
    }
  }

  bool plot = false;
  for (int i = 0; i < 128; i++)
  {
    for (int j = 0; j < 128; j++)
    {
      if (transition_sum_map[i][j] > 200)
        plot = true;
    }
  }

  if (plot)
  {
    Size patternsize(4, 4); //number of centers
    std::vector<cv::Point2f> centers; //this will be filled by the detected centers
    std::vector<cv::Point2f> centers_tmp; //this will be filled by the detected centers
    std::vector<int> center_count;

    cv::Mat img(128, 128, CV_8U);
    img = cv::Scalar(255);

    for (int i = 0; i < 128; i++)
    {
      for (int j = 0; j < 128; j++)
      {
        img.at<uint8_t>(j, i) = 255 - transition_sum_map[i][j];
        if (transition_sum_map[i][j] > 10)
        {
          bool found_center = false;
          for (int k = 0; k < centers.size(); k++)
          {
            float dist_x = i - centers[k].x;
            float dist_y = j - centers[k].y;
            if (dist_x * dist_x + dist_y * dist_y < 10 * 10)
            {
              centers_tmp[k].x += transition_sum_map[i][j] * i;
              centers_tmp[k].y += transition_sum_map[i][j] * j;
              center_count[k] += transition_sum_map[i][j];
              found_center = true;
            }
          }
          if (!found_center)
          {
            centers.push_back(cv::Point2f(i, j));
            centers_tmp.push_back(cv::Point2f(transition_sum_map[i][j] * i, transition_sum_map[i][j] * j));
            center_count.push_back(transition_sum_map[i][j]);
          }
        }
      }
    }

    if (centers.size() == 16)
    {
      for (int i = 0; i < centers.size(); i++)
      {
        centers[i].x = centers_tmp[i].x / center_count[i];
        centers[i].y = centers_tmp[i].y / center_count[i];
        ROS_ERROR("center %i: %f %f", i, centers[i].x, centers[i].y);
      }

      CirclesGridClusterFinder grid(false);

      std::vector<cv::Point2f> centers_good;
      grid.findGrid(centers, patternsize, centers_good);

      ROS_ERROR("centers: %i", centers_good.size());

      cv::Mat img2;
      cv::cvtColor(img, img2, CV_GRAY2RGB);
      cv::drawChessboardCorners(img2, patternsize, Mat(centers_good), true);

      cv::imshow("img", img2);
      cv::waitKey(10);
    }

    reset_maps();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dvs_calibration");

  ros::NodeHandle nh;

  // load parameters from rosparam
  int dots = 4;

  Pattern p;
  cv::imshow("test", p.get_intrinsic_calibration_pattern(45.0/180.0*M_PI, 0.0));
  cv::waitKey(0);
  cv::imshow("test", p.get_window_outline_pattern());
  cv::waitKey(0);
  cv::imshow("test", p.get_focus_adjustment_pattern());
  cv::waitKey(0);

  // register callback
//  ros::Subscriber sub = nh.subscribe("dvs_events", 1, eventsCallback);
//
//  ros::spin();

  return 0;
}

#include "ros/ros.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include "dvs_calibration/DvsCalibration.h"

using namespace cv;
using namespace std;


void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  if (msg->events.size() > 1000)
  {
    cv::Mat img(128, 128, CV_8U);
    img = cv::Scalar(255);

    for (int i = 0; i < msg->events.size(); ++i)
    {
      int x = msg->events[i].x;
      int y = msg->events[i].y;

      img.at<uint8_t>(y, x) = 0;
    }

    cv::GaussianBlur(img, img, cv::Size(5,5), 0, 0);

    Size patternsize(4,4); //number of centers
    std::vector<Point2f> centers; //this will be filled by the detected centers

    bool patternfound = cv::findCirclesGrid(img, patternsize, centers, CALIB_CB_SYMMETRIC_GRID);

    cv::Mat img2;
    cv::cvtColor(img, img2, CV_GRAY2RGB);
    cv::drawChessboardCorners(img2, patternsize, Mat(centers), patternfound);

    cv::imshow("img", img2);
    cv::waitKey(10);
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "dvs_calibration");

  ros::NodeHandle nh;

  // load parameters from rosparam
  int dots = 7;

  DVSCalibration calib(dots);

  // register callback
  ros::Subscriber sub = nh.subscribe("dvs_events", 1, eventsCallback);

  ros::spin();

  return 0;
}

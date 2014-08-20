#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>

#include "dvs_calibration/circlesgrid.hpp"
#include "dvs_calibration/board_detection.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseStamped.h>

int threshold = 250;
const double dot_distance = 0.05;
const int dots = 5;

image_transport::Publisher visualizationPublisher;
ros::Publisher cameraPosePublisher;

bool gotCameraInfo = false;
sensor_msgs::CameraInfo cameraInfo;

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  gotCameraInfo = true;
  cameraInfo = *msg;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  if (!gotCameraInfo)
  {
    ROS_WARN("%s: Camera info not yet received...", ros::this_node::getName().c_str());
  }

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

  // threshodl and invert
  cv::Mat thresholded_image;
  cv::threshold(cv_ptr->image, thresholded_image, threshold, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<cv::Point2f> centers;

  cv::findContours( thresholded_image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); //Find the Contour BLOBS
  for( int i = 0; i < contours.size(); i++ )
  {
    cv::Moments mu = moments( cv::Mat(contours[i]), false );
    cv::Point2f center = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00);
    if (center.x > 10 && center.y > 10)
      centers.push_back(center);
  }

  std::vector<cv::Point2f> centers_good;
  cv::Size patternsize(dots, dots);
  CirclesGridClusterFinder grid(false);
  grid.findGrid(centers, patternsize, centers_good);

  cv_bridge::CvImage cv_ptr_visu;
  cv_ptr->image.copyTo(cv_ptr_visu.image);
  cv_ptr_visu.encoding = "bgr8";
  cvtColor(cv_ptr_visu.image, cv_ptr_visu.image, CV_GRAY2RGB);

  if (centers_good.size() == dots * dots)
  {
    cv::drawChessboardCorners(cv_ptr_visu.image, cv::Size(dots, dots), cv::Mat(centers_good), true);

    // estimate pose
    std::vector<cv::Point3f> world_pattern;
    for (int i = 0; i < dots; i++)
    {
      for (int j = 0; j < dots; j++)
      {
        world_pattern.push_back(cv::Point3f(i * dot_distance , j * dot_distance , 0.0));
      }
    }

    // if we have camera info, also publish pose
    if (gotCameraInfo)
    {
      cv::Mat rvec, tvec;
      cv::Mat cameraMatrix(3, 3, CV_64F);
      cv::Mat distCoeffs(1, 5, CV_64F);

      // convert to OpenCV
      for (int i = 0; i < 5; i++)
        distCoeffs.at<double>(i) = cameraInfo.D[i];
      for (int i = 0; i < 9; i++)
        cameraMatrix.at<double>(i) = cameraInfo.K[i];

      cv::solvePnP(world_pattern, centers_good, cameraMatrix, distCoeffs, rvec, tvec);

      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.stamp = ros::Time::now();
      pose_msg.header.frame_id = "camera";
      pose_msg.pose.position.x = tvec.at<double>(0);
      pose_msg.pose.position.y = tvec.at<double>(1);
      pose_msg.pose.position.z = tvec.at<double>(2);

      double angle = cv::norm(rvec);
      cv::normalize(rvec, rvec);
      pose_msg.pose.orientation.x = rvec.at<double>(0) * sin(angle/2.0);
      pose_msg.pose.orientation.y = rvec.at<double>(1) * sin(angle/2.0);
      pose_msg.pose.orientation.z = rvec.at<double>(2) * sin(angle/2.0);
      pose_msg.pose.orientation.w = cos(angle/2.0);

      cameraPosePublisher.publish(pose_msg);
    }
  }

  visualizationPublisher.publish(cv_ptr_visu.toImageMsg());
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "camera_pose");

  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  visualizationPublisher = it.advertise("camera_pose/visualization", 1);

  cameraPosePublisher = nh.advertise<geometry_msgs::PoseStamped>("camera_pose/pose", 1);

  ros::param::get("~threshold", threshold);

  ros::Subscriber imageSub = nh.subscribe("image", 1, imageCallback);
  ros::Subscriber cameraInfoSub = nh.subscribe("camera_info", 1, cameraInfoCallback);

  ros::spin();

  return 0;
}

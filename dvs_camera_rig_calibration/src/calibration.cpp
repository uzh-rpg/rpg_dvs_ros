#include "ros/ros.h"

//#include <visp_hand2eye_calibration/compute_effector_camera_quick.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

#include <Eigen/Dense>

#include <eigen_conversions/eigen_msg.h>

ros::ServiceClient callHandEyeCalibrationSrv;

ros::Publisher poseDifferencePub;

//visp_hand2eye_calibration::compute_effector_camera_quick calibSrv;

double max_time_difference = 0.1;

Eigen::Vector3d relative_sum = Eigen::Vector3d::Zero();
int num_measurements = 0;

geometry_msgs::PoseStamped lastHandPose;
void handCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  lastHandPose = *msg;
}

void eyeCallback(const geometry_msgs::PoseStamped::ConstPtr& eyePose)
{
  if (eyePose->header.stamp - lastHandPose.header.stamp < ros::Duration(max_time_difference))
  {
    // convert to eigen
    Eigen::Affine3d handPoseEigen, eyePoseEigen;
    tf::poseMsgToEigen(lastHandPose.pose, handPoseEigen);
    tf::poseMsgToEigen(eyePose->pose, eyePoseEigen);

    // invert poses
    geometry_msgs::Transform handTransform, eyeTransform;
    tf::transformEigenToMsg(handPoseEigen.inverse(), handTransform);
    tf::transformEigenToMsg(eyePoseEigen.inverse(), eyeTransform);
//    calibSrv.request.world_effector.transforms.push_back(handTransform);
//    calibSrv.request.camera_object.transforms.push_back(eyeTransform);

    // compute relative pose between hand and eye
    Eigen::Affine3d relativePoseEigen = handPoseEigen * eyePoseEigen.inverse();

    geometry_msgs::PoseStamped relative_pose;
    tf::poseEigenToMsg(relativePoseEigen, relative_pose.pose);
    poseDifferencePub.publish(relative_pose);
//    ROS_INFO("%f %f %f %f", relative_pose.pose.position.x, relative_pose.pose.position.y, relative_pose.pose.position.z, relative_pose.pose.orientation.w);

    // running average
    relative_sum += relativePoseEigen.translation();
    num_measurements++;

    ROS_INFO("Running average (n=%d): %2.6f %2.6f %2.6f [m]", num_measurements,
             relative_sum.x()/num_measurements, relative_sum.y()/num_measurements,
             relative_sum.z()/num_measurements);

//    ROS_INFO("Added another measurement.");
//
//    if (calibSrv.request.camera_object.transforms.size() > 20)
//    {
//      if (callHandEyeCalibrationSrv.call(calibSrv))
//      {
//        ROS_INFO("Trafo is %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f",
//                 calibSrv.response.effector_camera.translation.x,
//                 calibSrv.response.effector_camera.translation.y,
//                 calibSrv.response.effector_camera.translation.z,
//                 calibSrv.response.effector_camera.rotation.x,
//                 calibSrv.response.effector_camera.rotation.y,
//                 calibSrv.response.effector_camera.rotation.z,
//                 calibSrv.response.effector_camera.rotation.w);
//      }
//      else
//      {
//        ROS_ERROR("Error calling hand-eye calibration service.");
//      }
//    }
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dvs_renderer");

  ros::NodeHandle nh;

  // get parameters
//  ros::param::get("~max_time_difference", max_time_difference);

  // setup subscribers and publishers
  ros::Subscriber handSub = nh.subscribe("hand_pose", 1, handCallback);
  ros::Subscriber eyeSub = nh.subscribe("eye_pose", 1, eyeCallback);

  poseDifferencePub = nh.advertise<geometry_msgs::PoseStamped>("relative_pose", 1);

//  callHandEyeCalibrationSrv = nh.serviceClient<visp_hand2eye_calibration::compute_effector_camera_quick>("compute_effector_camera_quick");

  ros::spin();

  return 0;
}

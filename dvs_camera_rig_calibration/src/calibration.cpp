#include "ros/ros.h"

#include <visp_hand2eye_calibration/compute_effector_camera_quick.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>

ros::ServiceClient callHandEyeCalibrationSrv;

visp_hand2eye_calibration::compute_effector_camera_quick calibSrv;

double max_time_difference = 0.1;

geometry_msgs::Transform poseToTrafo(geometry_msgs::Pose p)
{
  geometry_msgs::Transform t;
  t.translation.x = p.position.x;
  t.translation.y = p.position.y;
  t.translation.z = p.position.z;

  t.rotation.x = p.orientation.x;
  t.rotation.y = p.orientation.y;
  t.rotation.z = p.orientation.z;
  t.rotation.w = p.orientation.w;

  return t;
}

geometry_msgs::PoseStamped lastHandPose;
void handCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  lastHandPose = *msg;
}

void eyeCallback(const geometry_msgs::PoseStamped::ConstPtr& eyePose)
{
  if (eyePose->header.stamp - lastHandPose.header.stamp < ros::Duration(max_time_difference))
  {
    calibSrv.request.world_effector.transforms.push_back(poseToTrafo(lastHandPose.pose));
    calibSrv.request.camera_object.transforms.push_back(poseToTrafo(eyePose->pose));

    ROS_INFO("Added another measurement.");

    if (calibSrv.request.camera_object.transforms.size() > 20)
    {
      if (callHandEyeCalibrationSrv.call(calibSrv))
      {
        ROS_INFO("Trafo is %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f",
                 calibSrv.response.effector_camera.translation.x,
                 calibSrv.response.effector_camera.translation.y,
                 calibSrv.response.effector_camera.translation.z,
                 calibSrv.response.effector_camera.rotation.x,
                 calibSrv.response.effector_camera.rotation.y,
                 calibSrv.response.effector_camera.rotation.z,
                 calibSrv.response.effector_camera.rotation.w);
      }
      else
      {
        ROS_ERROR("Error calling hand-eye calibration service.");
      }
    }
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

  callHandEyeCalibrationSrv = nh.serviceClient<visp_hand2eye_calibration::compute_effector_camera_quick>("compute_effector_camera_quick");

  ros::spin();

  return 0;
}

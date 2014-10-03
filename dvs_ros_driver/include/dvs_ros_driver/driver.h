#include <ros/ros.h>

// boost
#include <boost/thread.hpp>
#include <boost/thread/thread_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// messages
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

// DVS driver
#include <dvs_driver/dvs_driver.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dvs_ros_driver/DVS_ROS_DriverConfig.h>

// camera info manager
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

namespace dvs_ros_driver {

class DvsRosDriver {
public:
  DvsRosDriver();
  ~DvsRosDriver();

private:
  void change_dvs_parameters();
  void callback(dvs_ros_driver::DVS_ROS_DriverConfig &config, uint32_t level);
  void readout();

  ros::NodeHandle nh;
  ros::Publisher event_array_pub;
  ros::Publisher camera_info_pub;
  dvs::DVS_Driver *driver;

  boost::thread* parameter_thread;
  boost::thread* readout_thread;

  boost::posix_time::time_duration delta;

  dvs_ros_driver::DVS_ROS_DriverConfig current_config;
  camera_info_manager::CameraInfoManager* cameraInfoManager;

  bool parameter_update_required;

};

} // namespace

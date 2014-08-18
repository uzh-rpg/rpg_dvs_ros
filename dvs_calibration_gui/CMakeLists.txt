cmake_minimum_required(VERSION 2.8.3)
project(dvs_calibration_gui)

find_package(catkin REQUIRED COMPONENTS )
catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs 
)
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

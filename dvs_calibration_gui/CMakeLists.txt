cmake_minimum_required(VERSION 2.8.3)
project(dvs_calib_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)
catkin_package()
catkin_python_setup()

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/dvs_calib_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
cmake_minimum_required(VERSION 2.8.3)
project(dvs_ros_driver)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  libcaer_catkin
  dvs_msgs
  std_msgs
  dynamic_reconfigure
  camera_info_manager
  nodelet
)

generate_dynamic_reconfigure_options(
  cfg/DVS_ROS_Driver.cfg
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

# since we need eigen and boost search them as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(Boost REQUIRED COMPONENTS system thread)

# export the dependencis of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS 
  	include
  CATKIN_DEPENDS 
  	roscpp 
    libcaer_catkin
  	dvs_msgs 
  	std_msgs
  	dynamic_reconfigure
  	camera_info_manager
  	nodelet
  DEPENDS 
  	boost
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# make the executable
add_executable(dvs_ros_driver
  src/driver_node.cpp
  src/driver.cpp
)

# make the nodelet into a library
add_library(dvs_ros_driver_nodelet 
  src/driver_nodelet.cpp
  src/driver.cpp
)

# to build the executable we depend on other packets, 
# they need to be build beforehand, especially the messages
add_dependencies(dvs_ros_driver 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_gencfg
)

add_dependencies(dvs_ros_driver_nodelet 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_gencfg
)

# link the executable to the necesarry libs
target_link_libraries(dvs_ros_driver
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CATKIN_DEVEL_PREFIX}/lib/libcaer${CMAKE_SHARED_LIBRARY_SUFFIX}
)

# link the executable to the necesarry libs
target_link_libraries(dvs_ros_driver_nodelet
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${CATKIN_DEVEL_PREFIX}/lib/libcaer${CMAKE_SHARED_LIBRARY_SUFFIX}
)

# Install the nodelet library
install(TARGETS dvs_ros_driver_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install other support files for installation
install(FILES dvs_ros_driver_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

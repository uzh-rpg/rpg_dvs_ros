cmake_minimum_required(VERSION 2.8.3)
project(dvs_ros_driver)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dvs_driver
  dvs_msgs
  std_msgs
  dynamic_reconfigure
  camera_info_manager
)

generate_dynamic_reconfigure_options(
  cfg/DVS_Driver.cfg
)

# since we need eigen and boost search them as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(Boost REQUIRED COMPONENTS system thread)

# export the dependencis of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  	roscpp 
  	dvs_driver 
  	dvs_msgs 
  	std_msgs
  	dynamic_reconfigure
  	camera_info_manager
  DEPENDS boost
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# make the executable
add_executable(dvs_ros_driver
  src/driver_node.cpp src/driver.cpp
)

# to build the executable we depend on other packets, 
# they need to be build beforehand, especially the messages
add_dependencies(dvs_ros_driver ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencf)

# link the executable to the necesarry libs
target_link_libraries(dvs_ros_driver
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

cmake_minimum_required(VERSION 2.8.3)
project(dvs_camera_rig_calibration)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visp_hand2eye_calibration
  geometry_msgs
  eigen_conversions
  cmake_modules
)
# since we need eigen and boost search them as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(Eigen REQUIRED)

# export the dependencis of this package for who ever depends on us
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  	roscpp
	visp_hand2eye_calibration
	geometry_msgs
	eigen_conversions
  DEPENDS OpenCV
)

# tell catkin where to find the headers for this project
include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

# make the executable
add_executable(calibration_handler
  src/calibration.cpp
)

# to build the executable we depend on other packets, 
# they need to be build beforehand, especially the messages
add_dependencies(calibration_handler  ${catkin_EXPORTED_TARGETS})

# link the executable to the necesarry libs
target_link_libraries(calibration_handler
   ${catkin_LIBRARIES}
)

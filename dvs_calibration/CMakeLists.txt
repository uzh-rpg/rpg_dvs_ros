cmake_minimum_required(VERSION 2.8.3)
project(dvs_calibration)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  dvs_msgs
  geometry_msgs
  sensor_msgs
)

# since we need eigen and boost search them as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)

# export the dependencis of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp 
    image_transport 
    cv_bridge 
    dvs_msgs
    geometry_msgs
    sensor_msgs
  DEPENDS eigen OpenCV
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# make the executable
add_executable(calibration
  src/calibration.cpp
  src/dvs_calibration.cpp
  src/pattern.cpp
  src/circlesgrid.cpp
)

# to build the executable we depend on other packets, 
# they need to be build beforehand, especially the messages
add_dependencies(calibration ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# link the executable to the necesarry libs
target_link_libraries(calibration
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

cmake_minimum_required(VERSION 2.8.3)
project(dvs_calibration)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  dvs_msgs
  geometry_msgs
  sensor_msgs
  cmake_modules
)

# since we need boost search it as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(OpenCV 2 REQUIRED)
find_package(Boost REQUIRED)

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
  DEPENDS
  	OpenCV
  	boost
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


#########################
# make the executable for MONO
#########################

add_executable(mono
  src/mono_node.cpp
  src/mono_dvs_calibration.cpp
  src/dvs_calibration.cpp
  src/board_detection.cpp
  src/transition_map.cpp
)

# to build the executable we depend on other packets,
# they need to be build beforehand, especially the messages
add_dependencies(mono ${catkin_EXPORTED_TARGETS})

# link the executable to the necesarry libs
target_link_libraries(mono
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)


#########################
# make the executable for STEREO
#########################

add_executable(stereo
  src/stereo_node.cpp
  src/stereo_dvs_calibration.cpp
  src/dvs_calibration.cpp
  src/board_detection.cpp
  src/transition_map.cpp
)

# to build the executable we depend on other packets,
# they need to be build beforehand, especially the messages
add_dependencies(stereo ${catkin_EXPORTED_TARGETS})

# link the executable to the necesarry libs
target_link_libraries(stereo
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)


#########################
# make the executable for Camera-DVS
#########################

add_executable(camera_dvs
  src/camera_dvs_node.cpp
  src/camera_dvs_calibration.cpp
  src/dvs_calibration.cpp
  src/board_detection.cpp
  src/transition_map.cpp
)

# to build the executable we depend on other packets,
# they need to be build beforehand, especially the messages
add_dependencies(camera_dvs ${catkin_EXPORTED_TARGETS})

# link the executable to the necesarry libs
target_link_libraries(camera_dvs
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

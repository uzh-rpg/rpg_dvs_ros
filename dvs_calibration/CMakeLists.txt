cmake_minimum_required(VERSION 2.8.3)
project(dvs_calibration)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)


#########################
# make the executable for MONO
#########################

cs_add_executable(mono
  src/mono_node.cpp
  src/mono_dvs_calibration.cpp
  src/dvs_calibration.cpp
  src/circlesgrid.cpp
  src/board_detection.cpp
  src/transition_map.cpp
)

# link the executable to the necesarry libs
target_link_libraries(mono
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)


#########################
# make the executable for STEREO
#########################

cs_add_executable(stereo
  src/stereo_node.cpp
  src/stereo_dvs_calibration.cpp
  src/dvs_calibration.cpp
  src/circlesgrid.cpp
  src/board_detection.cpp
  src/transition_map.cpp
)

# link the executable to the necesarry libs
target_link_libraries(stereo
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)


#########################
# make the executable for Camera-DVS
#########################

cs_add_executable(camera_dvs
  src/camera_dvs_node.cpp
  src/camera_dvs_calibration.cpp
  src/dvs_calibration.cpp
  src/circlesgrid.cpp
  src/board_detection.cpp
  src/transition_map.cpp
)

# link the executable to the necesarry libs
target_link_libraries(camera_dvs
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

# Export package
cs_export()

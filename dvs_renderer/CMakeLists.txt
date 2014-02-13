cmake_minimum_required(VERSION 2.8.3)
project(dvs_renderer)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  dvs_msgs
  cv_bridge
  image_transport
)
# since we need eigen and boost search them as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(OpenCV REQUIRED)

# export the dependencis of this package for who ever depends on us
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs dvs_msgs cv_bridge image_transport
  DEPENDS OpenCV
)

# tell catkin where to find the headers for this project
include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# make the executable
add_executable(dvs_renderer
  src/renderer.cpp
)

# to build the executable we depend on other packets, 
# they need to be build beforehand, especially the messages
add_dependencies(dvs_renderer  ${catkin_EXPORTED_TARGETS})

# link the executable to the necesarry libs
target_link_libraries(dvs_renderer
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

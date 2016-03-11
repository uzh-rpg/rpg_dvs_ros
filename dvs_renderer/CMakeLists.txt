cmake_minimum_required(VERSION 2.8.3)
project(dvs_renderer)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  dvs_msgs
  cv_bridge
  image_transport
  nodelet
)

# since we need eigen and boost search them as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(OpenCV REQUIRED)

# export the dependencis of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    dvs_msgs 
    cv_bridge 
    image_transport
    nodelet
  DEPENDS 
    OpenCV
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# make the executable
add_executable(dvs_renderer
  src/image_tracking.cpp
  src/renderer.cpp
  src/renderer_node.cpp
)

# make the nodelet into a library
add_library(dvs_renderer_nodelet
  src/image_tracking.cpp
  src/renderer_nodelet.cpp
  src/renderer.cpp
)

# to build the executable we depend on other packets, 
# they need to be build beforehand, especially the messages
add_dependencies(dvs_renderer 
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(dvs_renderer_nodelet
  ${catkin_EXPORTED_TARGETS}
)

# link the executable to the necesarry libs
target_link_libraries(dvs_renderer
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

target_link_libraries(dvs_renderer_nodelet
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

# Install the nodelet library
install(TARGETS dvs_renderer_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install other support files for installation
install(FILES dvs_renderer_nodelet.xml nodelet_stereo.launch nodelet_mono.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

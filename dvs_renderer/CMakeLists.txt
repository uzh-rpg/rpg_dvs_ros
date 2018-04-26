cmake_minimum_required(VERSION 2.8.3)
project(dvs_renderer)

find_package(catkin_simple REQUIRED)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(OpenCV REQUIRED)

# make the executable
cs_add_executable(dvs_renderer
  src/image_tracking.cpp
  src/renderer.cpp
  src/renderer_node.cpp
)

# make the nodelet into a library
cs_add_library(dvs_renderer_nodelet
  src/image_tracking.cpp
  src/renderer_nodelet.cpp
  src/renderer.cpp
)

# link the executable to the necessary libs
target_link_libraries(dvs_renderer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(dvs_renderer_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# install the executable
install(
  TARGETS dvs_renderer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install other support files for installation
install(FILES dvs_renderer_nodelet.xml launch/nodelet_stereo.launch launch/nodelet_mono.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cs_install()
cs_export()

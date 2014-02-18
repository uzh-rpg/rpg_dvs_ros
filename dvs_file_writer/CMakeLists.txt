cmake_minimum_required(VERSION 2.8.3)
project(dvs_file_writer)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dvs_msgs
)
# export the dependencis of this package for who ever depends on us
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp dvs_msgs
)

# tell catkin where to find the headers for this project
include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)

# make the executable
add_executable(dvs_file_writer
  src/writer.cpp
)

# to build the executable we depend on other packets, 
# they need to be build beforehand, especially the messages
add_dependencies(dvs_file_writer ${catkin_EXPORTED_TARGETS})

# link the executable to the necesarry libs
target_link_libraries(dvs_file_writer
   ${catkin_LIBRARIES}
)

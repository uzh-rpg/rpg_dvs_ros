cmake_minimum_required(VERSION 2.8.3)
project(dvs_file_writer)

find_package(catkin_simple REQUIRED)

catkin_simple()

# make the executable
cs_add_executable(dvs_file_writer
  src/writer.cpp
)

# link the executable to the necesarry libs
target_link_libraries(dvs_file_writer
   ${catkin_LIBRARIES}
)

# Export package
cs_export()

cmake_minimum_required(VERSION 2.8.3)
project(dvs_driver)

# search for everything we need to build the package
find_package(catkin)

# since we need eigen and boost search them as well
find_package(Boost REQUIRED COMPONENTS thread)

include(FindPkgConfig)
pkg_check_modules(LIBUSB1 REQUIRED libusb-1.0)

# export the dependencis of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include ${LIBUSB1_INCLUDE_DIRS}
  DEPENDS libusb-1.0
  LIBRARIES dvs_driver
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${LIBUSB1_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# make the library
add_library(dvs_driver
  src/dvs_driver.cpp
)

target_link_libraries(dvs_driver
  ${LIBUSB1_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS dvs_driver
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project(libcaer_catkin)

find_package(catkin_simple REQUIRED)

catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/libcaer)

ExternalProject_Add(libcaer_src
  GIT_REPOSITORY https://github.com/inilabs/libcaer.git
  GIT_TAG 0d819fb6b77b2891f6a3ad2266622afcb9f53509
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX} -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INSTALL_LIBDIR=${CATKIN_DEVEL_PREFIX}/lib
  BUILD_COMMAND make
  INSTALL_COMMAND make install
)

cs_install()
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/libcaer
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/libcaer/
  CFG_EXTRAS libcaer-extras.cmake)

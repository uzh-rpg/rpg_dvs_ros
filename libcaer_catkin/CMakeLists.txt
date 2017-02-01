cmake_minimum_required(VERSION 2.8.3)
project(libcaer_catkin)

find_package(catkin_simple REQUIRED)

catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/libcaer)

ExternalProject_Add(libcaer_src
  GIT_REPOSITORY https://github.com/inilabs/libcaer.git
  GIT_TAG 57466910da84c25231aab57e99f246239722a628
  UPDATE_COMMAND ""
  PATCH_COMMAND patch -p1 -t -N < ${CMAKE_CURRENT_SOURCE_DIR}/libcaer-startup.patch
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

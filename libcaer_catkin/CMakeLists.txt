cmake_minimum_required(VERSION 2.8.3)
project(libcaer_catkin)

find_package(catkin_simple REQUIRED)

catkin_simple()

include(ExternalProject)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/libcaer)

ExternalProject_Add(libcaer_src
  GIT_REPOSITORY https://gitlab.com/inivation/libcaer.git
  GIT_TAG 5d93b61d88b79a6ad66cca94300b9cd7e614f5f5
  UPDATE_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CATKIN_DEVEL_PREFIX} -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_SHARED_LIBS=ON -DUDEV_INSTALL=0
  BUILD_COMMAND make
  INSTALL_COMMAND make install
)

cs_add_library(${PROJECT_NAME} src/dependency_tracker.cc)
add_dependencies(${PROJECT_NAME} libcaer_src)
target_link_libraries(${PROJECT_NAME}
  ${CATKIN_DEVEL_PREFIX}/lib/libcaer${CMAKE_SHARED_LIBRARY_SUFFIX})

cs_install()

cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/
  CFG_EXTRAS libcaer-extras.cmake)

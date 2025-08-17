cmake_minimum_required(VERSION 3.0.2)
project(obstacle_util)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  camera_util
  sdf_map
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES obstacle_util
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(LIBS
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  ${PCL_LIBRARIES}
  -lboost_system
)

link_directories(
  ${LIBS}
  lib
)

add_library( obstacle_util
  src/obstacle_util.cc
  src/kalman_filter.cc
  src/DBSCANKdtreeCluster.cc
)

target_link_libraries( obstacle_util
  ${LIBS}
) 
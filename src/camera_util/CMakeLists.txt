cmake_minimum_required(VERSION 3.0.2)
project(camera_util)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES camera_util
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
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

add_library( camera_util
  src/camera_util.cc
)

target_link_libraries( camera_util
  ${LIBS}
) 

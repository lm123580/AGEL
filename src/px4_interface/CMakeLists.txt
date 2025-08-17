cmake_minimum_required(VERSION 3.0.2)
project(px4_interface)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES px4_interface
 CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

set(LIBS
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBS}
  -lboost_system
)

link_directories(
  ${LIBS}
  lib
)

add_library( px4_interface
  src/px4_interface.cc
)

target_link_libraries( px4_interface
  ${LIBS}
) 

add_executable( px4_node
  src/px4_node.cc
  src/px4_interface.cc
)

target_link_libraries( px4_node
  ${LIBS}
) 
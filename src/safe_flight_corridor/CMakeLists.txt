cmake_minimum_required(VERSION 3.0.2)
project(safe_flight_corridor)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sdf_map
  decomp_ros_utils
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES safe_flight_corridor
  CATKIN_DEPENDS roscpp sdf_map decomp_ros_utils
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

add_library( safe_flight_corridor
  src/safe_flight_corridor.cc
)

target_link_libraries( safe_flight_corridor
  ${LIBS}
) 
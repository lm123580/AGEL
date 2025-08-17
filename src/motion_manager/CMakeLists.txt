cmake_minimum_required(VERSION 3.0.2)
project(motion_manager)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  visualization_msgs
  map_manager
  safe_flight_corridor
  path_searcher
  bspline_util
  px4_interface
  quadrotor_dynamic
  mpcc
  frontier_util
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES motion_manager
 CATKIN_DEPENDS roscpp nav_msgs visualization_msgs map_manager safe_flight_corridor path_searcher bspline_util px4_interface quadrotor_dynamic mpcc frontier_util
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
  ${LIBS} lib
)

add_library( motion_manager
  src/motion_manager.cc
)

target_link_libraries( motion_manager
  ${LIBS}
) 
cmake_minimum_required(VERSION 3.0.2)
project(mpcc)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  bspline_util
  quadrotor_dynamic
  roscpp
  map_manager
  sdf_map
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mpcc
 CATKIN_DEPENDS bspline_util quadrotor_dynamic roscpp map_manager sdf_map
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
  /usr/local/lib/libnlopt.so
  -lboost_system
)


add_library( mpcc
  src/mpcc.cc
)

target_link_libraries( mpcc
  ${LIBS}
) 
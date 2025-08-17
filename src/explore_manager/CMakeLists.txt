cmake_minimum_required(VERSION 3.0.2)
project(explore_manager)


set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(Eigen3 3.1.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  sdf_map
  map_manager
  frontier_util
  px4_interface
  motion_manager
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES explore_manager
 CATKIN_DEPENDS roscpp visualization_msgs sdf_map map_manager frontier_util px4_interface motion_manager
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

add_library( explore_manager
  src/explore_manager.cc
)

target_link_libraries( explore_manager
  ${LIBS}
) 
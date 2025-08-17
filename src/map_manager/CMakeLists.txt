cmake_minimum_required(VERSION 3.0.2)
project(map_manager)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV QUIET)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  visualization_msgs
  camera_util
  sdf_map
  obstacle_util
  frontier_util
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES map_manager
 CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp sensor_msgs visualization_msgs frontier_util
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(LIBS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${PCL_LIBRARIES}
  -lboost_system
)

link_directories(
  ${LIBS}
  lib
)

add_library( map_manager
  src/map_manager.cc
)

target_link_libraries( map_manager
  ${LIBS}
) 

# add_executable( test_node
#   src/test_node.cc
#   src/map_manager.cc
# )

# target_link_libraries( test_node
#   ${LIBS}
# ) 


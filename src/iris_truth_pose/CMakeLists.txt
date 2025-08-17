cmake_minimum_required(VERSION 3.0.2)
project(iris_truth_pose)


set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_ros
  geometry_msgs
  roscpp
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES iris_truth_pose
 CATKIN_DEPENDS gazebo_msgs gazebo_ros geometry_msgs roscpp
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


set(LIBS
  ${catkin_LIBRARIES}
  -lboost_system
)


link_directories(
  ${LIBS}
  lib
)


add_executable( iris_truth_pose
  src/iris_truth_pose.cc
)


target_link_libraries( iris_truth_pose
  ${LIBS}
) 
cmake_minimum_required(VERSION 3.0.2)
project(system_fsm)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  map_manager
  motion_manager
  explore_manager
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES system_fsm
 CATKIN_DEPENDS roscpp geometry_msgs map_manager motion_manager explore_manager std_msgs
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


add_library( system_fsm
  src/system_fsm.cc
)

target_link_libraries( system_fsm
  ${LIBS}
) 


add_executable( system_fsm_node
  src/start_node.cc
  src/system_fsm.cc
)

target_link_libraries( system_fsm_node
  ${LIBS}
) 
cmake_minimum_required(VERSION 3.0.2)
project(quadrotor_dynamic)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 3.1.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES quadrotor_dynamic
 CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
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

add_library( quadrotor_dynamic
  src/quadrotor_dynamic.cc
)

target_link_libraries( quadrotor_dynamic
  ${LIBS}
) 

# add_executable( qua_dynamic_node
#   src/qua_dynamic_node.cc
#   src/quadrotor_dynamic.cc
# )

# target_link_libraries( qua_dynamic_node
#   ${LIBS}
# ) 

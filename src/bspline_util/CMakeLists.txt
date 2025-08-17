cmake_minimum_required(VERSION 3.0.2)
project(bspline_util)


set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(Eigen3 3.1.0 REQUIRED)
find_package(NLopt REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  visualization_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bspline_util
  CATKIN_DEPENDS geometry_msgs roscpp visualization_msgs
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


link_directories(
  ${LIBS} lib
)


add_library( bspline_util
  src/uniform_bspline.cc
  src/bspline_opt.cc
)

target_link_libraries( bspline_util
  ${LIBS}
) 


# add_executable( b_node
#   src/b_node.cc
#   src/uniform_bspline.cc
# )

# target_link_libraries( b_node
#   ${LIBS}
# ) 
cmake_minimum_required(VERSION 3.0.2)
project(path_searcher)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(Eigen3 3.1.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp sdf_map
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_searcher
 CATKIN_DEPENDS roscpp
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
   lib
)


add_library( path_searcher
  src/astar.cc
)

target_link_libraries( path_searcher
  ${LIBS}
) 
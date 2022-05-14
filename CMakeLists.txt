cmake_minimum_required(VERSION 3.5.1)
project(ros-static-graph-analysis)

find_package(Clang REQUIRED)
set(LLVM_LINK_COMPONENTS support)
set(CMAKE_CXX_STANDARD 17)

# clang plugin library for ROS extraction
add_library(FindROSPrimitivesPlugin MODULE src/FindROSPrimitivesPlugin.cpp)
target_include_directories(FindROSPrimitivesPlugin PRIVATE
  ${CLANG_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/include
)


cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcd_segmentation)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SRC_LIST pcd_segmentation.cpp OpenNI_Motor.cpp)
add_executable (pcd_segmentation ${SRC_LIST})
target_link_libraries (pcd_segmentation ${PCL_LIBRARIES})


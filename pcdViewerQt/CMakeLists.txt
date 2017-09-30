cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcdViewerQt)

find_package(PCL  REQUIRED)
find_package(VTK  REQUIRED)
find_package(Qt5Widgets REQUIRED)

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable (pcdViewerQt main.cpp)
target_link_libraries (pcdViewerQt ${PCL_LIBRARIES} )

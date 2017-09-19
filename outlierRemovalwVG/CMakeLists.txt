cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(outlierRemovalwVG)

find_package(PCL  REQUIRED)
find_package(VTK  REQUIRED)

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable (outlierRemovalwVG main.cpp)
target_link_libraries (outlierRemovalwVG ${PCL_LIBRARIES} )

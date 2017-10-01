cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(tranICPRegistrationQt)

find_package(PCL  REQUIRED)
find_package(VTK  REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package( Boost REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS})

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable (tranICPRegistrationQt main.cpp)
target_link_libraries (tranICPRegistrationQt ${PCL_LIBRARIES} )
qt5_use_modules(tranICPRegistrationQt Core Gui Widgets)



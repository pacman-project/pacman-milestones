cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(parts)

find_package(PCL 1.7 REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(include /usr/include/python3.2)
link_directories(/usr/include/python3.2)

add_executable (comp vision.cpp main.cpp)
target_link_libraries ( comp ${PCL_LIBRARIES} armadillo python3.2mu tinyxml)


cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(icp_pacman_pe)

# Eigen 3
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread date_time)
INCLUDE_DIRECTORIES(contrib/tclap-1.2.1/include)
include_directories(${PCL_INCLUDE_DIRS}
                   ${BOOST_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (icp_pacman_pe src/icp_pacman_pe.cpp)
target_link_libraries (icp_pacman_pe ${PCL_LIBRARIES} ${BOOST_LIBRARIES} )

INSTALL(TARGETS
        icp_pacman_pe
        RUNTIME DESTINATION bin
        BUNDLE DESTINATION bin)

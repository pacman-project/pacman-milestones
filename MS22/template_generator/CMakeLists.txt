cmake_minimum_required(VERSION 2.8.11)

# Project name
project(template_generator)

if(CMAKE_COMPILER_IS_GNUCXX)
    #add_definitions(-std=gnu++0x)
endif()

# Eigen 3
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#PCL for visualization of frames ...
find_package(PCL 1.2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system thread date_time)
include_directories(${PCL_INCLUDE_DIRS}
                    ${BOOST_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("/home/alexander/Downloads/GT/WildMagic5/SDK/Include")
link_directories("/home/alexander/Downloads/GT/WildMagic5/SDK/Library/ReleaseDynamic")

set(WM5_LIBRARIES Wm5Applications Wm5Graphics Wm5Imagics Wm5Physics Wm5Mathematics Wm5Core pthread m)

add_executable(template_generator src/template_generator.cpp src/forwardKinematics.cpp)
target_link_libraries(template_generator ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${WM5_LIBRARIES} ${BOOST_LIBRARIES} )

#add_executable(part_roi_generator part_roi_generator.cpp forwardKinematics.cpp)
#target_link_libraries(part_roi_generator ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} )


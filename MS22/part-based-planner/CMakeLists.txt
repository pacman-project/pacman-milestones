PROJECT (part-based-planner)

# Project parameters:

OPTION(PBP_USE_KTH_KUKA "Link to KTH Kuka drivers")
OPTION(PBP_USE_SDH "Link to SDH driver")
OPTION(PBP_STATIC "Build static executable")
OPTION(PBP_USE_MOVEIT "Link to moveit")

# Build system:

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECT_NAMESPACE pbp)
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

FILE(GLOB PBP_HEADERS lib-src/${PROJECT_NAMESPACE}/*.hpp)
FILE(GLOB PBP_CODE lib-src/*.cpp)
FILE(GLOB PBP_BIN_HEADERS bin-src/${PROJECT_NAMESPACE}/*.hpp)
FILE(GLOB PBP_BIN_CODE bin-src/*.cpp)
FILE(GLOB PBP_SCRIPTS scripts/*.py)


IF(APPLE)
  SET(EXECUTABLE_FLAG MACOSX_BUNDLE)
ENDIF(APPLE)

IF(PBP_STATIC)
  SET(LIBLINK_FLAG STATIC)
  ADD_DEFINITIONS(-DPBP_STATIC)
ELSE(PBP_STATIC)
  SET(LIBLINK_FLAG SHARED)
ENDIF(PBP_STATIC)

ADD_DEFINITIONS(-Wall)
ADD_DEFINITIONS(-Wno-c++11-extensions -Wno-unused-variable -Wno-unneeded-internal-declaration)

# It is important that this line comes before any other INCLUDE_DIRECTORIES:
INCLUDE_DIRECTORIES(lib-src)
INCLUDE_DIRECTORIES(bin-src)
INCLUDE_DIRECTORIES(contrib/tclap-1.2.1/include)

SET(PROJECT_LIBS ${PROJECT_LIBS} z m pthread)

IF(PBP_USE_KTH_KUKA)
  ADD_DEFINITIONS(-DPBP_USE_KTH_ROBOT)
  SET_SOURCE_FILES_PROPERTIES(lib-src/KukaReachable.cpp COMPILE_FLAGS -std=c++0x)
  SET(PROJECT_LIBS buolacore robot blackbox ${PROJECT_LIBS})
ENDIF(PBP_USE_KTH_KUKA)

IF(PBP_USE_SDH)
  ADD_DEFINITIONS(-DPBP_USE_SDH)
  SET(PROJECT_LIBS SDHLibrary-CPP ${PROJECT_LIBS})
ENDIF(PBP_USE_SDH)

IF(PBP_USE_MOVEIT)
  ADD_DEFINITIONS(-DPBP_USE_MOVEIT)
  LINK_DIRECTORIES(/home/demo/catkin_ws/devel/lib/)
  SET(PROJECT_LIBS  roscpp rostime moveit_move_group_interface ${PROJECT_LIBS})
ENDIF(PBP_USE_MOVEIT)

INCLUDE(FindPkgConfig)

# CPPFLAGS
# Although CMake reads LDFLAGS from the shell's env, it still ignores CPPFLAGS

STRING(REGEX REPLACE "-I([^ ]+)" "\\1" CPPFLAGS "$ENV{CPPFLAGS}")
SEPARATE_ARGUMENTS(CPPFLAGS)
INCLUDE_DIRECTORIES(${CPPFLAGS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#UIBK INCLUDES
add_definitions(-DOROCOS_TARGET=gnulinux)
find_package(moveit_core REQUIRED)
find_package(roscpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(eigen REQUIRED)
include_directories(
  ${orocos_kdl_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  /usr/include/eigen3
)
link_directories("/home/alexander/Downloads/GT/WildMagic5/SDK/Library/ReleaseDynamic")
include_directories("/home/alexander/Downloads/GT/WildMagic5/SDK/Include")
set(WM5_LIBRARIES Wm5Applications Wm5Graphics Wm5Imagics Wm5Physics Wm5Mathematics Wm5Core pthread m)

#add_executable(test_uibk_kinematics src/test_UIBK_kinematics.cpp src/KukaLWR_Kinematics.cpp)
#target_link_libraries(test_uibk_kinematics ${orocos_kdl_LIBRARIES})
SET(PROJECT_LIBS ${PROJECT_LIBS} ${orocos_kdl_LIBRARIES} ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES} ${roscpp_LIBRARIES} ${WM5_LIBRARIES})

#END OF UIBK INCLUDES


SET(NUKLEI_LIB_NAME nuklei)
PKG_CHECK_MODULES(NUKLEI ${NUKLEI_LIB_NAME})
INCLUDE_DIRECTORIES(${NUKLEI_INCLUDE_DIRS})
LINK_DIRECTORIES(${NUKLEI_LIBRARY_DIRS})
ADD_DEFINITIONS(${NUKLEI_CFLAGS})
SET(PROJECT_LIBS ${NUKLEI_LIBRARIES} ${PROJECT_LIBS})

# IDE

SOURCE_GROUP(bin REGULAR_EXPRESSION bin-src/*)
SOURCE_GROUP(lib REGULAR_EXPRESSION lib-src/*)
SOURCE_GROUP(scripts REGULAR_EXPRESSION scripts/*)
#SOURCE_GROUP(bin FILES ${PBP_BIN_HEADERS} ${PBP_BIN_CODE})
#SOURCE_GROUP(lib FILES ${PBP_HEADERS} ${PBP_CODE})
#SOURCE_GROUP(scripts FILES ${PBP_SCRIPTS})


# Poducts

ADD_EXECUTABLE(${PROJECT_NAMESPACE}-plan ${EXECUTABLE_FLAG}
               bin-src/plan.cpp)
               
ADD_LIBRARY(${PROJECT_NAMESPACE} ${LIBLINK_FLAG} ${PBP_HEADERS} ${PBP_CODE})

# Link

TARGET_LINK_LIBRARIES(${PROJECT_NAMESPACE}
                      ${PROJECT_LIBS})
TARGET_LINK_LIBRARIES(${PROJECT_NAMESPACE}-plan
                      ${PROJECT_NAMESPACE} ${PROJECT_LIBS})

IF(PBP_STATIC)
  SET_TARGET_PROPERTIES(${PROJECT_NAMESPACE}-plan
                        PROPERTIES LINK_FLAGS "-static")
ENDIF(PBP_STATIC)

# Install

INSTALL(FILES ${PBP_HEADERS} DESTINATION include/${PROJECT_NAMESPACE})
INSTALL(TARGETS ${PROJECT_NAMESPACE}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
INSTALL(TARGETS
        ${PROJECT_NAMESPACE}-plan
        RUNTIME DESTINATION bin
        BUNDLE DESTINATION bin)
INSTALL(PROGRAMS scripts/pbp.py
                 scripts/pbp-planner.py
                 scripts/pbp-view.py
                 scripts/pbp-ulg-grasp.py
        DESTINATION bin)

cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(chai_devices_redis_driver)

# CMAKE OPTIONS
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
# SET(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)

# - jsoncpp
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
    find_library(JSONCPP_LIBRARY jsoncpp)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(JSONCPP jsoncpp)
    include_directories(${JSONCPP_INCLUDE_DIRS})
    link_directories(${JSONCPP_LIBRARY_DIRS})
    set(JSONCPP_LIBRARY ${JSONCPP_LIBRARIES})
endif ()

# - eigen3
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_INCLUDE_DIR)
    find_package(Eigen3 QUIET)
    # TODO: Find automatic way to find for Ubuntu 14.04
    if(NOT EIGEN3_INCLUDE_DIR)
        set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
    endif()
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

# - CHAI3D
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  add_definitions(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  add_definitions(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
link_directories(${CHAI3D_LIBRARY_DIRS})

# create headers list
set(chai_devices_redis_driver_INCLUDE_DIRS 
    ${JSONCPP_INCLUDE_DIRS}
)

# BUILD SOURCES AND LIBRARY
set(SOURCES 
    haptic_driver_chai.cpp
    timer/LoopTimer.cpp
    redis/RedisClient.cpp
)

# Create a library
add_executable(${PROJECT_NAME} ${SOURCES})

# create libraries list
set(chai_devices_redis_driver_LIBRARIES 
    ${CHAI3D_LIBRARIES}
    ${HIREDIS_LIBRARY}
    ${JSONCPP_LIBRARY}
)

include_directories( ${chai_devices_redis_driver_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME} ${chai_devices_redis_driver_LIBRARIES})


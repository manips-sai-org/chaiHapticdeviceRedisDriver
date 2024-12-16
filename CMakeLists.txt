cmake_minimum_required(VERSION 3.10)

project(chai_devices_redis_driver)

# CMAKE OPTIONS
set(CMAKE_BUILD_TYPE Release)

SET(CMAKE_CXX_FLAGS "-std=c++17 -I/usr/include -I/usr/local/include -fPIC")
if (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/opt/homebrew/include")
endif ()

# - CHAI3D
find_package(CHAI3D REQUIRED)
include_directories(${CHAI3D_INCLUDE_DIRS})
add_definitions(${CHAI3D_DEFINITIONS})
link_directories(${CHAI3D_LIBRARY_DIRS})

# include Eigen
find_package(Eigen3 QUIET)
include_directories(${EIGEN3_INCLUDE_DIR})

# include sai-common
FIND_PACKAGE(SAI-COMMON REQUIRED)
include_directories(${SAI-COMMON_INCLUDE_DIRS})

# BUILD SOURCES AND LIBRARY
set(SOURCES 
    haptic_driver_chai.cpp
)

# Create a library
add_executable(${PROJECT_NAME} ${SOURCES})

# create libraries list
set(chai_devices_redis_driver_LIBRARIES 
    ${CHAI3D_LIBRARIES}
    ${SAI-COMMON_LIBRARIES}
)

include_directories( ${chai_devices_redis_driver_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME} ${chai_devices_redis_driver_LIBRARIES})

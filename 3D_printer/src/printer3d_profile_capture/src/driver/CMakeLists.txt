# Pre-requisites about cmake itself
cmake_minimum_required(VERSION 3.5)

# Project name and the type of project
project(gocatorSensor)

# DEBUG/RELEASE
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RELEASE")
endif()

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find point cloud Library
find_package(PCL REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Set GO_SDK include paths (That could be part of a FindGocator.cmake)
set(GO_SDK_4 ${CMAKE_CURRENT_SOURCE_DIR}/../../external/GO_SDK)
find_path(
    GOCATOR_INCLUDES
    NAMES GoSdk/GoSdk.h
    PATHS ${GO_SDK_4}/Gocator/GoSdk)
find_path(
    KAPI_INCLUDES
    NAMES kApi/kApi.h
    PATHS ${GO_SDK_4}/Platform/kApi)
include_directories(${GOCATOR_INCLUDES} ${KAPI_INCLUDES})

# Set GO_SDK libs (That could be part of a FindGocator.cmake)
find_library(
    GOCATOR_LIBRARIES
    NAMES GoSdk
    PATHS ${GO_SDK_4}/lib/linux_x64d/)
find_library(
    KAPI_LIBRARIES
    NAMES kApi
    PATHS ${GO_SDK_4}/lib/linux_x64d/)

# Set source files
set(SRCS gocatorSensor.cpp)

# Set header files
set(HDRS gocatorSensor.h)

#Build library
add_library(${PROJECT_NAME} SHARED ${SRCS})
target_link_libraries(${PROJECT_NAME} ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

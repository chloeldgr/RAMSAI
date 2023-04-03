cmake_minimum_required(VERSION 3.5)
project(ros2_3d_printer_profile_capture)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2_gocator_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Find point cloud Library
FIND_PACKAGE(PCL REQUIRED COMPONENTS)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

# Set GO_SDK include paths (That could be part of a FindGocator.cmake)
SET(GO_SDK_4 ${CMAKE_CURRENT_SOURCE_DIR}/external/GO_SDK) 
FIND_PATH(
    GOCATOR_INCLUDES
    NAMES GoSdk/GoSdk.h
    PATHS ${GO_SDK_4}/Gocator/GoSdk)
FIND_PATH(
    KAPI_INCLUDES
    NAMES kApi/kApi.h
    PATHS ${GO_SDK_4}/Platform/kApi)
INCLUDE_DIRECTORIES(${GOCATOR_INCLUDES} ${KAPI_INCLUDES})

# Set GO_SDK libs (That could be part of a FindGocator.cmake)
FIND_LIBRARY(
    GOCATOR_LIBRARIES
    NAMES GoSdk
    PATHS ${GO_SDK_4}/lib/linux_x64d) 
FIND_LIBRARY(
    KAPI_LIBRARIES
    NAMES kApi
    PATHS ${GO_SDK_4}/lib/linux_x64d) 

# Set source files 
SET(LIB_SRCS 
  src/driver/gocatorSensor.cpp)

# Set header files 
SET(LIB_HDRS 
  src/driver/gocatorSensor.h)

INCLUDE_DIRECTORIES(${LIB_HDRS})

# Install the python module for this package
# ament_python_install_package(script/)
# ament_python_install_package(nodes/)



################################################################################
# Declare ROS messages, services and actions
################################################################################

add_executable( gocator_sensor_node src/nodes/gocatorSensorNode.cpp ${LIB_HDRS} ${LIB_SRCS} )
target_link_libraries (gocator_sensor_node ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES} ${Boost_SYSTEM_LIBRARY})

ament_target_dependencies(gocator_sensor_node rclcpp sensor_msgs std_msgs ros2_gocator_msgs pcl_conversions)


# Install C++ headers
#install(
#  DIRECTORY ${PROJECT_NAME}/include
#            ${LIB_DIR}/include
#  DESTINATION include/${PROJECT_NAME}
#  FILES_MATCHING PATTERN "*.hpp"
#)

install(TARGETS
  gocator_sensor_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Install python scripts
# install(
#   DIRECTORY external/GO_SDK/lib/linux_x64d/ #launch param script
#   DESTINATION share/${PROJECT_NAME}/external/GO_SDK/lib/linux_x64d/
# )

# install(
#   PROGRAMS
#     script/tcpSocket.py
#     script/udpSocket.py
#     nodes/iiwa_command_node.py
#     DESTINATION lib/${PROJECT_NAME}
# )

ament_package()

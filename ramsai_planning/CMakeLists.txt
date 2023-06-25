cmake_minimum_required(VERSION 3.8)
project(ramsai_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_hybrid_planning REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(iiwa_description REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  moveit_hybrid_planning
  moveit_servo
  tf2_geometry_msgs
  moveit_ros_planning
  pluginlib
  Eigen3
  Boost
  control_msgs
)

include_directories(include)

add_executable(robot_cartesian_ompl
  src/robot_cartesian_ompl.cpp
  src/helper_tools.hpp
)

add_executable(robot_cartesian_pilz
  src/robot_cartesian_pilz.cpp
  src/helper_tools.hpp
)

add_executable(robot_cartesian_hybrid_planning
  src/robot_cartesian_hybrid_planning.cpp
  src/helper_tools.hpp
)

add_executable(cancel_hybrid_planning
  src/cancel_hybrid_planning.cpp
)

add_library(servo_local_solver SHARED src/servo_solver.cpp)
add_library(servo_sampler SHARED src/servo_sampler.cpp)
add_library(motion_planning_pipeline SHARED src/global_planning_pipeline.cpp)

ament_target_dependencies(robot_cartesian_ompl PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(robot_cartesian_pilz PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_target_dependencies(robot_cartesian_hybrid_planning PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_target_dependencies(cancel_hybrid_planning PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_target_dependencies(servo_local_solver ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_target_dependencies(servo_sampler ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_target_dependencies(motion_planning_pipeline ${THIS_PACKAGE_INCLUDE_DEPENDS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS
    robot_cartesian_ompl
    robot_cartesian_pilz
    robot_cartesian_hybrid_planning
    cancel_hybrid_planning
    servo_local_solver
    servo_sampler
    motion_planning_pipeline
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(moveit_hybrid_planning plugins.xml)

ament_export_libraries(
  servo_local_solver
  servo_sampler
  motion_planning_pipeline
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()

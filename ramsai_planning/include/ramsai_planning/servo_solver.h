#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/local_planner/local_constraint_solver_interface.h>
#include "tf2_ros/transform_broadcaster.h"

#include <moveit_servo/servo.h>

namespace ramsai_planning
{
class ServoSolver : public moveit::hybrid_planning::LocalConstraintSolverInterface
{
public:
  bool initialize(const rclcpp::Node::SharedPtr& node,
                  const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                  const std::string& group_name) override;
  bool reset() override;

  moveit_msgs::action::LocalPlanner::Feedback
  solve(const robot_trajectory::RobotTrajectory& local_trajectory,
        const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> local_goal,
        trajectory_msgs::msg::JointTrajectory& local_solution) override;

private:
  rclcpp::Node::SharedPtr node_handle_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  moveit_servo::ServoParameters::SharedConstPtr servo_parameters_;
  double velocity_scaling_threshold_;  // Indicates below which velocity scaling replanning should be triggered

  // Servo cpp interface
  std::unique_ptr<moveit_servo::Servo> servo_;

  // Inteface to communicate with servo
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr ee_tf_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_cmd_pub_;
  bool publish_ = true;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr collision_velocity_scale_sub_;

  // Subscribe to laser corrections
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr laser_corrections_sub_;
  double laser_correction_ = 0;

  // Flag to indicate if replanning is necessary
  bool replan_;

  // Flag to indicate that replanning is requested
  bool feedback_send_;
};
}  // namespace ramsai_planning
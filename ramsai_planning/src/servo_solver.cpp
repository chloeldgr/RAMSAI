#include <ramsai_planning/servo_solver.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace ramsai_planning
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");

bool ServoSolver::initialize(const rclcpp::Node::SharedPtr& node,
                             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                             const std::string& group_name)
{
  // Load parameter & initialize member variables
  if (node->has_parameter("velocity_scaling_threshold"))
    node->get_parameter<double>("velocity_scaling_threshold", velocity_scaling_threshold_);
  else
    velocity_scaling_threshold_ = node->declare_parameter<double>("velocity_scaling_threshold", 0.0);

  planning_scene_monitor_ = planning_scene_monitor;
  node_handle_ = node;
  
  twist_cmd_pub_ = node_handle_->create_publisher<geometry_msgs::msg::TwistStamped>("~/delta_twist_cmds", 10);
  joint_cmd_pub_ = node_handle_->create_publisher<control_msgs::msg::JointJog>("~/delta_joint_cmds", 10);
  ee_tf_pub_ = node_handle_->create_publisher<geometry_msgs::msg::TransformStamped>("~/eef_position", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_handle_);

  // Get Servo Parameters
  servo_parameters_ = moveit_servo::ServoParameters::makeServoParameters(node_handle_, "moveit_servo");
  if (!servo_parameters_)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
    return false;
  }

  // Create Servo and start it
  servo_ = std::make_unique<moveit_servo::Servo>(node_handle_, servo_parameters_, planning_scene_monitor_);
  servo_->start();

  return true;
}

bool ServoSolver::reset()
{
  RCLCPP_INFO(LOGGER, "Reset Servo Solver");
  return true;
};

moveit_msgs::action::LocalPlanner::Feedback
ServoSolver::solve(const robot_trajectory::RobotTrajectory& local_trajectory,
                   const std::shared_ptr<const moveit_msgs::action::LocalPlanner::Goal> local_goal,
                   trajectory_msgs::msg::JointTrajectory& local_solution)
{
  // Create Feedback
  moveit_msgs::action::LocalPlanner::Feedback feedback_result;

  // Transform next robot trajectory waypoint into JointJog message
  moveit_msgs::msg::RobotTrajectory robot_command;
  local_trajectory.getRobotTrajectoryMsg(robot_command);

  auto msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  msg->header.stamp = node_handle_->now();

  if (!robot_command.joint_trajectory.points.empty())
  {
    const auto current_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();

    for (int i=0; i < local_trajectory.getWayPointCount();i++) 
    {
      moveit::core::RobotState state(local_trajectory.getRobotModel());
      state.setVariablePositions(robot_command.joint_trajectory.joint_names,
                                      robot_command.joint_trajectory.points[i].positions);
      state.update();
      Eigen::Isometry3d pose = state.getFrameTransform("tool0");
      Eigen::Quaterniond q(pose.linear());
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = node_handle_->get_clock()->now();
      t.header.frame_id = "iiwa_base";
      t.child_frame_id = "local_traj" + std::to_string(i);
      t.transform.translation.x = pose.translation().x();
      t.transform.translation.y = pose.translation().y();
      t.transform.translation.z = pose.translation().z();
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(t);
    }

    moveit::core::RobotState target_state(local_trajectory.getRobotModel());
    target_state.setVariablePositions(robot_command.joint_trajectory.joint_names,
                                      robot_command.joint_trajectory.points[0].positions);
    target_state.update();

    // TF planning_frame -> current EE
    Eigen::Isometry3d current_pose = current_state->getFrameTransform("tool0");
    // std::cout << "current tran = " << current_pose.translation() << std::endl;
    // TF planning -> target EE
    Eigen::Isometry3d target_pose = target_state.getFrameTransform("tool0");
    // std::cout << "target tran = " << target_pose.translation() << std::endl;

    // current EE -> planning frame * planning frame -> target EE
    Eigen::Isometry3d diff_pose = current_pose.inverse() * target_pose;
    Eigen::AngleAxisd axis_angle_current(current_pose.linear());
    Eigen::AngleAxisd axis_angle_target(target_pose.linear());

    constexpr double fixed_vel = 0.05;
    const double vel_scale = fixed_vel / (target_pose.translation() - current_pose.translation()).norm();

    msg->twist.linear.x = (target_pose.translation().x() - current_pose.translation().x()) * vel_scale;
    msg->twist.linear.y = (target_pose.translation().y() - current_pose.translation().y()) * vel_scale;
    msg->twist.linear.z = (target_pose.translation().z() - current_pose.translation().z()) * vel_scale;
    msg->twist.angular.x = (axis_angle_target.axis().x() * axis_angle_target.angle() - axis_angle_current.axis().x() * axis_angle_current.angle()) * vel_scale;
    msg->twist.angular.y = (axis_angle_target.axis().y() * axis_angle_target.angle() - axis_angle_current.axis().y() * axis_angle_current.angle()) * vel_scale;
    msg->twist.angular.z = (axis_angle_target.axis().z() * axis_angle_target.angle() - axis_angle_current.axis().z() * axis_angle_current.angle()) * vel_scale;
    // Rotation joint is laser correction, not from delta-position
    // msg->twist.angular.z = 0;  // laser_correction_;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node_handle_->get_clock()->now();
    t.header.frame_id = "iiwa_base";
    t.child_frame_id = "local_target";
    Eigen::Quaterniond qt(target_pose.linear());
    t.transform.translation.x = target_pose.translation().x();
    t.transform.translation.y = target_pose.translation().y();
    t.transform.translation.z = target_pose.translation().z();
    t.transform.rotation.x = qt.x();
    t.transform.rotation.y = qt.y();
    t.transform.rotation.z = qt.z();
    t.transform.rotation.w = qt.w();
    tf_broadcaster_->sendTransform(t);
  }

  twist_cmd_pub_->publish(std::move(msg));

  // Publish EEF Position
  geometry_msgs::msg::TransformStamped tf;
  if (servo_->getEEFrameTransform(tf)) {
    tf.child_frame_id = "servo_frame";
    ee_tf_pub_->publish(tf);
    tf_broadcaster_->sendTransform(tf);
  }

  return feedback_result;
}
}  // namespace ramsai_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ramsai_planning::ServoSolver, moveit::hybrid_planning::LocalConstraintSolverInterface);
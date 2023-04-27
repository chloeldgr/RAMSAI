#include <ramsai_planning/servo_sampler.h>

#include <moveit/kinematic_constraints/utils.h>

namespace ramsai_planning
{
constexpr double WAYPOINT_RADIAN_TOLERANCE = 0.2;  // rad: L1-norm sum for all joints
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");

bool ServoSampler::initialize(const rclcpp::Node::SharedPtr& node,
                                    const moveit::core::RobotModelConstPtr& robot_model, const std::string& group_name)
{
  // Load parameter & initialize member variables
  if (node->has_parameter("pass_through"))
  {
    node->get_parameter<bool>("pass_through", pass_through_);
  }
  else
  {
    pass_through_ = node->declare_parameter<bool>("pass_through", false);
  }
  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group_name);
  next_waypoint_index_ = 0;
  return true;
}

moveit_msgs::action::LocalPlanner::Feedback
ServoSampler::addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory)
{
  // Reset trajectory operator to delete old reference trajectory
  reset();

  // Throw away old reference trajectory and use trajectory update
  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(new_trajectory);

  // Parametrize trajectory and calculate velocity and accelerations
  time_parametrization_.computeTimeStamps(*reference_trajectory_);

  // Return empty feedback
  return feedback_;
}

bool ServoSampler::reset()
{
  // Reset index
  next_waypoint_index_ = 0;
  reference_trajectory_->clear();
  return true;
}
moveit_msgs::action::LocalPlanner::Feedback
ServoSampler::getLocalTrajectory(const moveit::core::RobotState& current_state,
                                       robot_trajectory::RobotTrajectory& local_trajectory)
{
  // Delete previous local trajectory
  local_trajectory.clear();

  // Determine current local trajectory based on configured behavior
  if (pass_through_)
  {
    // Use reference_trajectory as local trajectory
    local_trajectory.append(*reference_trajectory_, 0.0, 0, reference_trajectory_->getWayPointCount());
  }
  else
  {
    // Get next desired robot state
    moveit::core::RobotState next_desired_goal_state = reference_trajectory_->getWayPoint(next_waypoint_index_);

    // Check if state reached
    auto sum_joint_distance = next_desired_goal_state.distance(current_state);
    // double pos_distance = (current_state.getFrameTransform("tool0").translation() -
    //                        next_desired_goal_state.getFrameTransform("tool0").translation())
    //                           .norm();
    if (sum_joint_distance <= WAYPOINT_RADIAN_TOLERANCE)
    {
      // Update index (and thus desired robot state)
      next_waypoint_index_ += 1;
    }

    // Construct local trajectory containing the next global trajectory waypoint
    for (size_t i = next_waypoint_index_; i < reference_trajectory_->getWayPointCount(); ++i)
    {
      local_trajectory.addSuffixWayPoint(reference_trajectory_->getWayPoint(i),
                                         reference_trajectory_->getWayPointDurationFromPrevious(i));
    }
  }

  // Return empty feedback
  return feedback_;
}

double ServoSampler::getTrajectoryProgress(const moveit::core::RobotState& current_state)
{
  // Check if trajectory is unwinded
  if (next_waypoint_index_ >= reference_trajectory_->getWayPointCount() - 1)
  {
    return 1.0;
  }
  return 0.0;
}
}  // namespace ramsai_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ramsai_planning::ServoSampler, moveit::hybrid_planning::TrajectoryOperatorInterface);
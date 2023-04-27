#include <moveit/local_planner/trajectory_operator_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace ramsai_planning
{
class ServoSampler : public moveit::hybrid_planning::TrajectoryOperatorInterface
{
public:
  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
                  const std::string& group_name) override;
  moveit_msgs::action::LocalPlanner::Feedback
  addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory) override;
  moveit_msgs::action::LocalPlanner::Feedback
  getLocalTrajectory(const moveit::core::RobotState& current_state,
                     robot_trajectory::RobotTrajectory& local_trajectory) override;
  double getTrajectoryProgress(const moveit::core::RobotState& current_state) override;
  bool reset() override;

private:
  std::size_t
      next_waypoint_index_;  // Indicates which reference trajectory waypoint is the current local goal constrained
  bool pass_through_;  // If true, the reference_trajectory is simply forwarded each time the getLocalTrajectory() function is called
  moveit_msgs::action::LocalPlanner::Feedback feedback_;  // Empty feedback
  trajectory_processing::TimeOptimalTrajectoryGeneration time_parametrization_;
};
}  // namespace ramsai_planning
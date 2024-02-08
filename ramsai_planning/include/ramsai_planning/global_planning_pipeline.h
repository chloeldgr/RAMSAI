#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/global_planner/global_planner_interface.h>

// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

namespace ramsai_planning
{
  class GlobalPlanningPipeline: public moveit::hybrid_planning::GlobalPlannerInterface
  {
public:
    GlobalPlanningPipeline() = default;
    ~GlobalPlanningPipeline() = default;
    bool initialize(const rclcpp::Node::SharedPtr & node) override;
    bool reset() noexcept override;
    moveit_msgs::msg::MotionPlanResponse
    plan(
      const std::shared_ptr < rclcpp_action::ServerGoalHandle < moveit_msgs::action::GlobalPlanner >> global_goal_handle)
    override;

private:
    rclcpp::Node::SharedPtr node_ptr_;
    std::shared_ptr < moveit_cpp::MoveItCpp > moveit_cpp_;
  };
}  // namespace ramsai_planning

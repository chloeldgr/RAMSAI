#include "ramsai_planning/global_planning_pipeline.h"
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("global_planner_component");
}

namespace ramsai_planning
{
const std::string PLANNING_SCENE_MONITOR_NS = "planning_scene_monitor_options.";
const std::string PLANNING_PIPELINES_NS = "planning_pipelines.";
const std::string PLAN_REQUEST_PARAM_NS = "plan_request_params.";
const std::string UNDEFINED = "<undefined>";

bool GlobalPlanningPipeline::initialize(const rclcpp::Node::SharedPtr& node)
{
  // Declare planning pipeline parameter
  node->declare_parameter<std::vector<std::string>>(PLANNING_PIPELINES_NS + "pipeline_names",
                                                    std::vector<std::string>({ "pilz" }));
  node->declare_parameter<std::string>(PLANNING_PIPELINES_NS + "namespace", UNDEFINED);

  // Default PlanRequestParameters. These can be overridden when plan() is called
  node->declare_parameter<std::string>(PLAN_REQUEST_PARAM_NS + "planner_id", "LIN");
  node->declare_parameter<std::string>(PLAN_REQUEST_PARAM_NS + "planning_pipeline", "pilz");
  node->declare_parameter<int>(PLAN_REQUEST_PARAM_NS + "planning_attempts", 5);
  node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "planning_time", 1.0);
  node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "max_velocity_scaling_factor", 1.0);
  node->declare_parameter<double>(PLAN_REQUEST_PARAM_NS + "max_acceleration_scaling_factor", 1.0);
  node->declare_parameter<std::string>(PLANNING_PIPELINES_NS + "planning_plugin",
                                       "pilz_industrial_motion_planner/CommandPlanner");

  // Planning Scene options
  node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "name", UNDEFINED);
  node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "robot_description", UNDEFINED);
  node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "joint_state_topic", UNDEFINED);
  node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "attached_collision_object_topic", UNDEFINED);
  node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "publish_planning_scene_topic", UNDEFINED);
  node->declare_parameter<std::string>(PLANNING_SCENE_MONITOR_NS + "monitored_planning_scene_topic", UNDEFINED);
  node->declare_parameter<double>(PLANNING_SCENE_MONITOR_NS + "wait_for_initial_state_timeout", 10.0);

  // Trajectory Execution Functionality (required by the GlobalPlanningPipeline but not used within hybrid planning)
  node->declare_parameter<std::string>("moveit_controller_manager", UNDEFINED);

  node_ptr_ = node;

  // Initialize MoveItCpp API
  moveit_cpp::MoveItCpp::Options moveit_cpp_options(node);
  moveit_cpp_options.planning_pipeline_options.parent_namespace = PLANNING_PIPELINES_NS;
  moveit_cpp_options.planning_pipeline_options.pipeline_names = { default_planning_pipeline };
  moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(node, moveit_cpp_options);

  return true;
}

bool GlobalPlanningPipeline::reset() noexcept
{
  // Do Nothing
  return true;
}

moveit_msgs::msg::MotionPlanResponse GlobalPlanningPipeline::plan(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> global_goal_handle)
{
  moveit_msgs::msg::MotionPlanResponse response;

  if ((global_goal_handle->get_goal())->motion_sequence.items.empty())
  {
    RCLCPP_WARN(LOGGER, "Global planner received motion sequence request with no items. At least one is needed.");
    response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return response;
  }

  // Process goal
  if ((global_goal_handle->get_goal())->motion_sequence.items.size() > 1)
  {
    RCLCPP_WARN(LOGGER, "Global planner received motion sequence request with more than one item but the "
                        "'moveit_planning_pipeline' plugin only accepts one item. Just using the first item as global "
                        "planning goal!");
  }
  auto motion_plan_req = (global_goal_handle->get_goal())->motion_sequence.items[0].req;

  // Set parameters required by the planning component
  moveit_cpp::PlanningComponent::PlanRequestParameters plan_params;
  plan_params.planner_id = motion_plan_req.planner_id;
  plan_params.planning_pipeline = motion_plan_req.pipeline_id;
  plan_params.planning_attempts = motion_plan_req.num_planning_attempts;
  plan_params.planning_time = motion_plan_req.allowed_planning_time;
  plan_params.max_velocity_scaling_factor = motion_plan_req.max_velocity_scaling_factor;
  plan_params.max_acceleration_scaling_factor = motion_plan_req.max_acceleration_scaling_factor;

  // Create planning component
  auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(motion_plan_req.group_name, moveit_cpp_);

  // Copy goal constraint into planning component
  planning_components->setGoal(motion_plan_req.goal_constraints);

  // Plan motion
  auto plan_solution = planning_components->plan(plan_params);
  if (!bool(plan_solution.error_code))
  {
    response.error_code = plan_solution.error_code;
    return response;
  }

  // Transform solution into MotionPlanResponse and publish it
  response.trajectory_start = plan_solution.start_state;
  response.group_name = motion_plan_req.group_name;
  plan_solution.trajectory->getRobotTrajectoryMsg(response.trajectory);
  response.error_code = plan_solution.error_code;

  return response;
}
}  // namespace ramsai_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ramsai_planning::GlobalPlanningPipeline, moveit::hybrid_planning::GlobalPlannerInterface);
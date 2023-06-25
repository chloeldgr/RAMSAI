// Copyright 2023 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Maciej Bednarczyk (mcbed.robotics@gmail.com)

#include <thread>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/action/hybrid_planner.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <rviz_visual_tools/rviz_visual_tools.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#if MOVEIT_ROLLING
#include <rclcpp/event_handler.hpp>
#elif MOVEIT_HUMBLE
#include <rclcpp/qos_event.hpp>
#endif
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/create_client.hpp>
#include "helper_tools.hpp"

using namespace std::chrono_literals;
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("robot_cartesian_hybrid_planning");
}  // namespace

class RobotCartesianHybridPlanning
{
public:
  RobotCartesianHybridPlanning(const rclcpp::Node::SharedPtr& node)
  {
    node_ = node;

    std::string hybrid_planning_action_name = "";
    if (node_->has_parameter("hybrid_planning_action_name"))
    {
      node_->get_parameter<std::string>("hybrid_planning_action_name", hybrid_planning_action_name);
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "hybrid_planning_action_name parameter was not defined");
      std::exit(EXIT_FAILURE);
    }
    hp_action_client_ =
        rclcpp_action::create_client<moveit_msgs::action::HybridPlanner>(node_, hybrid_planning_action_name);
  }

  void run()
  {
    RCLCPP_INFO(LOGGER, "Initialize Planning Scene Monitor");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node_, "robot_description", tf_buffer_, "planning_scene_monitor");
    if (!planning_scene_monitor_->getPlanningScene())
    {
      RCLCPP_ERROR(LOGGER, "The planning scene was not retrieved!");
      return;
    }
    else
    {
      planning_scene_monitor_->startStateMonitor();
      planning_scene_monitor_->providePlanningSceneService();  // let RViz display query PlanningScene
      planning_scene_monitor_->setPlanningScenePublishingFrequency(100);
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "/planning_scene");
      planning_scene_monitor_->startSceneMonitor();
    }

    if (!planning_scene_monitor_->waitForCurrentRobotState(node_->now(), 5))
    {
      RCLCPP_ERROR(LOGGER, "Timeout when waiting for /joint_states updates. Is the robot running?");
      return;
    }

    if (!hp_action_client_->wait_for_action_server(20s))
    {
      RCLCPP_ERROR(LOGGER, "Hybrid planning action server not available after waiting");
      return;
    }

    // Setup motion planning goal taken from motion_planning_api tutorial
    const std::string planning_group = "iiwa_arm";
    robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();

    // Create a RobotState and JointModelGroup
    const auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(planning_group);

    moveit::planning_interface::MoveGroupInterface move_group(node_, planning_group);

    // Configure a valid robot state
    robot_state->setToDefaultValues(joint_model_group, "ready");
    robot_state->update();
    // Lock the planning scene as briefly as possible
    {
      planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(planning_scene_monitor_);
      locked_planning_scene->setCurrentState(*robot_state);
    }

    // get waypoints from file
    std::string waypoint_file_path, waypoint_file_type;
    std::vector<double> shift;
    double scale;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    node_->get_parameter("waypoint_file_path", waypoint_file_path);
    node_->get_parameter("waypoint_file_type", waypoint_file_type);
    node_->get_parameter("shift", shift);
    node_->get_parameter("scale", scale);
    RCLCPP_INFO(LOGGER, "Loading waypoints from path: %s", waypoint_file_path.c_str());
    if (waypoint_file_type == "csv") {
      waypoints = csv2path(waypoint_file_path, shift, scale);
    } else if (waypoint_file_type == "apt") {
      waypoints = apt2path(waypoint_file_path, shift, scale);
    }

    // waypoints.resize(1);

    RCLCPP_INFO(LOGGER, "loaded %li waypoints", waypoints.size());

    bool plot_waypoints;
    bool plot_frames;
    node_->get_parameter("plot_waypoints", plot_waypoints);
    node_->get_parameter("plot_frames", plot_frames);

    // Visualize the plan in RViz
    namespace rvt = rviz_visual_tools;
    rviz_visual_tools::RvizVisualTools rvisual_tools("world", "path", node_);
    rvisual_tools.deleteAllMarkers();
    rvisual_tools.setBaseFrame("world");
    if (plot_waypoints) {
      rvisual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::XXXXSMALL);
    }
    if (plot_frames) {
      for (std::size_t i = 0; i < waypoints.size(); ++i) {
        rvisual_tools.publishAxisLabeled(waypoints[i], "", rvt::XXXXSMALL);
      }
    }
    rvisual_tools.trigger();
    rclcpp::sleep_for(2s);
    rvisual_tools.trigger();

    double velcocity_scaling, acceleration_scaling;
    node_->get_parameter("velcocity_scaling", velcocity_scaling);
    node_->get_parameter("acceleration_scaling", acceleration_scaling);

    // Setup
    std::vector<double> tolerance_pose(3, 1e-8);
    std::vector<double> tolerance_angle(3, 1e-8);

    // Goto starting point
    move_group.setPlanningPipelineId("pilz");
    move_group.setPlannerId("LIN");
    move_group.setPoseTarget(waypoints[0]);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      move_group.move();
    }
    move_group.setPoseTarget(waypoints[1]);
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      move_group.move();
    }

    RCLCPP_INFO(LOGGER, "Moving to starting point %s", success ? "" : "FAILED");

    // Create desired motion goal
    moveit_msgs::msg::MotionPlanRequest goal_motion_request;
    moveit_msgs::msg::MotionSequenceRequest sequence_request;
    moveit_msgs::msg::MotionSequenceItem sequence_item;
    moveit_msgs::msg::Constraints pose_goal;
    geometry_msgs::msg::PoseStamped pose;

    moveit::core::robotStateToRobotStateMsg(*robot_state, goal_motion_request.start_state);
    goal_motion_request.group_name = planning_group;
    goal_motion_request.num_planning_attempts = 10;
    goal_motion_request.max_velocity_scaling_factor = velcocity_scaling;
    goal_motion_request.max_acceleration_scaling_factor = acceleration_scaling;
    goal_motion_request.allowed_planning_time = 10.0;
    goal_motion_request.planner_id = "LIN";
    goal_motion_request.pipeline_id = "pilz";

    for (auto w = 2ul; w < waypoints.size(); w++) {
      pose.header.frame_id = "iiwa_base";
      pose.pose = waypoints[w];
      pose_goal = kinematic_constraints::constructGoalConstraints("tool0", pose);
      goal_motion_request.goal_constraints.clear();
      goal_motion_request.goal_constraints.push_back(pose_goal);
      goal_motion_request.start_state = moveit_msgs::msg::RobotState();

      sequence_item.req = goal_motion_request;
      sequence_item.blend_radius = 0.0;

      sequence_request.items.push_back(sequence_item);

    }

    auto goal_action_request = moveit_msgs::action::HybridPlanner::Goal();
    goal_action_request.planning_group = planning_group;
    goal_action_request.motion_sequence = sequence_request;

    auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SendGoalOptions();
    send_goal_options.result_callback =
        [](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::WrappedResult& result) {
          switch (result.code)
          {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(LOGGER, "Hybrid planning goal succeeded");
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(LOGGER, "Hybrid planning goal was aborted");
              return;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_ERROR(LOGGER, "Hybrid planning goal was canceled");
              return;
            default:
              RCLCPP_ERROR(LOGGER, "Unknown hybrid planning result code");
              return;
              RCLCPP_INFO(LOGGER, "Hybrid planning result received");
          }
        };
    send_goal_options.feedback_callback =
        [](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::SharedPtr& /*unused*/,
           const std::shared_ptr<const moveit_msgs::action::HybridPlanner::Feedback>& feedback) {
          RCLCPP_INFO_STREAM(LOGGER, feedback->feedback);
        };

    RCLCPP_INFO(LOGGER, "Sending hybrid planning goal");
    // Ask server to achieve some goal and wait until it's accepted
    auto goal_handle_future = hp_action_client_->async_send_goal(goal_action_request, send_goal_options);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SharedPtr hp_action_client_;
  rclcpp::Subscription<moveit_msgs::msg::MotionPlanResponse>::SharedPtr global_solution_subscriber_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("robot_cartesian_hybrid_planning", "", node_options);

  RobotCartesianHybridPlanning app(node);
  std::thread run_app([&app]() {
    app.run();
  });

  rclcpp::spin(node);
  run_app.join();
  rclcpp::shutdown();
  return 0;
}
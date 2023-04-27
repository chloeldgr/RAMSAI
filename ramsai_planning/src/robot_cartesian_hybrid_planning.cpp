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

using namespace std::chrono_literals;
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("test_hybrid_planning_client");
}  // namespace

class HybridPlanningDemo
{
public:
  HybridPlanningDemo(const rclcpp::Node::SharedPtr& node)
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
    const std::string planning_group = "ur_manipulator";
    robot_model_loader::RobotModelLoader robot_model_loader(node_, "robot_description");
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();

    // Create a RobotState and JointModelGroup
    const auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(planning_group);

    // Configure a valid robot state
    robot_state->setToDefaultValues(joint_model_group, "ready");
    robot_state->update();
    // Lock the planning scene as briefly as possible
    {
      planning_scene_monitor::LockedPlanningSceneRW locked_planning_scene(planning_scene_monitor_);
      locked_planning_scene->setCurrentState(*robot_state);
    }

    // Create desired motion goal
    moveit_msgs::msg::MotionPlanRequest goal_motion_request;

    moveit::core::robotStateToRobotStateMsg(*robot_state, goal_motion_request.start_state);
    goal_motion_request.group_name = planning_group;
    goal_motion_request.num_planning_attempts = 10;
    goal_motion_request.max_velocity_scaling_factor = 0.1;
    goal_motion_request.max_acceleration_scaling_factor = 0.1;
    goal_motion_request.allowed_planning_time = 2.0;
    goal_motion_request.planner_id = "ompl";
    goal_motion_request.pipeline_id = "ompl";

    moveit::core::RobotState goal_state1(robot_model);
    moveit::core::RobotState goal_state2(robot_model);
    std::vector<double> joint_values = { 180.0*3.1415/180, -52.0*3.1415/180, 49.0*3.1415/180, -87.0*3.1415/180, -90.0*3.1415/180, 10.0*3.1415/180 };
    goal_state1.setJointGroupPositions(joint_model_group, joint_values);
    joint_values[0] = 180.0*3.1415/180;
    goal_state2.setJointGroupPositions(joint_model_group, joint_values);
    // geometry_msgs::msg::Pose pose, pose2;
    // pose.position.x = 0.0;
    // pose.position.y = 0.7;
    // pose.position.z = 0.4;
    // pose.orientation.x = 1.0;
    // pose.orientation.y = 0.0;
    // pose.orientation.z = 0.0;
    // pose.orientation.w = 0.0;

    // pose2 = pose;
    // pose2.position.x = 0.0;

    // goal_state1.setFromIK(joint_model_group, pose);
    // goal_state2.setFromIK(joint_model_group, pose2);

    goal_motion_request.goal_constraints.resize(1);
    goal_motion_request.goal_constraints[0] =
        kinematic_constraints::constructGoalConstraints(goal_state1, joint_model_group);

    // Create Hybrid Planning action request
    moveit_msgs::msg::MotionSequenceItem sequence_item;
    sequence_item.req = goal_motion_request;
    sequence_item.blend_radius = 0.0;  // Single goal

    moveit_msgs::msg::MotionSequenceRequest sequence_request;
    sequence_request.items.push_back(sequence_item);

    goal_motion_request.goal_constraints[0] =
        kinematic_constraints::constructGoalConstraints(goal_state2, joint_model_group);
    // Create Hybrid Planning action request
    sequence_item.req = goal_motion_request;
    sequence_item.blend_radius = 0.0;  // Single goal

    sequence_request.items.push_back(sequence_item);

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

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("hybrid_planning_test_node", "", node_options);

  HybridPlanningDemo demo(node);
  std::thread run_demo([&demo]() {
    // This sleep isn't necessary but it gives humans time to process what's going on
    rclcpp::sleep_for(5s);
    demo.run();
  });

  rclcpp::spin(node);
  run_demo.join();
  rclcpp::shutdown();
  return 0;
}
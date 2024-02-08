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

#include <moveit_msgs/action/hybrid_planner.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>

namespace
{
using HPAction = moveit_msgs::action::HybridPlanner;
const rclcpp::Logger LOGGER = rclcpp::get_logger("hybrid_planning_cancellation");

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("hybrid_planning_cancellation", "", node_options);

  std::string hybrid_planning_action_name = "";
  if (node->has_parameter("hybrid_planning_action_name")) {
    node->get_parameter<std::string>("hybrid_planning_action_name", hybrid_planning_action_name);
  } else {
    RCLCPP_ERROR(LOGGER, "hybrid_planning_action_name parameter was not defined");
    std::exit(EXIT_FAILURE);
  }

  rclcpp_action::Client<HPAction>::SharedPtr client_ptr =
    rclcpp_action::create_client<HPAction>(node, hybrid_planning_action_name);
  if (!client_ptr->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }
  client_ptr->async_cancel_all_goals();
  RCLCPP_ERROR(node->get_logger(), "Cancellation was requested.");

  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}

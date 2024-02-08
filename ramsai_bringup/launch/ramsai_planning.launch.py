# Copyright 2022 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'base_frame_file',
            default_value='base_frame.yaml',
            description='Configuration file of robot base frame wrt World.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Start robot in Gazebo simulation.',
        )
    )

    # Initialize Arguments
    start_rviz = LaunchConfiguration('start_rviz')
    base_frame_file = LaunchConfiguration('base_frame_file')
    use_sim = LaunchConfiguration('use_sim')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ramsai_description'), 'config', 'ramsai.config.xacro']
            ),
            ' ',
            'prefix:=""',
            ' ',
            'base_frame_file:=',
            base_frame_file,
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    # Get SRDF via xacro
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare('ramsai_description'), 'srdf', 'iiwa.srdf.xacro']
            ),
            " ",
            "name:=",
            "iiwa_print",
        ]
    )

    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # Get planning parameters
    robot_description_planning_joint_limits = PathJoinSubstitution([
            FindPackageShare('iiwa_description'), "moveit2", "iiwa_joint_limits.yaml",
        ]
    )

    robot_description_planning_cartesian_limits = PathJoinSubstitution([
            FindPackageShare('iiwa_description'), "moveit2", "iiwa_cartesian_limits.yaml",
        ]
    )

    move_group_capabilities = {
        "capabilities": """pilz_industrial_motion_planner/MoveGroupSequenceAction \
            pilz_industrial_motion_planner/MoveGroupSequenceService"""
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare('iiwa_description'), "moveit2", "kinematics.yaml"]
    )

    planning_pipelines_config = PathJoinSubstitution([
            FindPackageShare('iiwa_description'), "moveit2", "planning_pipelines_config.yaml",
        ]
    )

    ompl_planning_config = PathJoinSubstitution([
            FindPackageShare('iiwa_description'), "moveit2", "ompl_planning.yaml",
        ]
    )

    moveit_controllers = PathJoinSubstitution(
        [FindPackageShare('iiwa_description'),
            "moveit2", "iiwa_moveit_controller_config.yaml"]
    )

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            planning_pipelines_config,
            ompl_planning_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": use_sim},
        ],
    )

    ##############################
    # Hybrid Planning
    ##############################

    # Load params
    common_hybrid_planning_param = PathJoinSubstitution([
        FindPackageShare("ramsai_description"),
        'moveit2',
        'common_hybrid_planning_params.yaml',
      ]
    )

    global_planner_param = PathJoinSubstitution([
        FindPackageShare("ramsai_description"),
        'moveit2',
        'global_planner.yaml',
      ]
    )

    local_planner_param = PathJoinSubstitution([
        FindPackageShare("ramsai_description"),
        'moveit2',
        'local_planner.yaml',
      ]
    )

    hybrid_planning_manager_param = PathJoinSubstitution([
        FindPackageShare("ramsai_description"),
        'moveit2',
        'hybrid_planning_manager.yaml',
      ]
    )

    servo_params = PathJoinSubstitution([
        FindPackageShare("ramsai_description"),
        'moveit2',
        'servo_config.yaml',
      ]
    )

    # Hybrid planner container
    planning_container = ComposableNodeContainer(
      name="hybrid_planning_container",
      namespace="/",
      package="rclcpp_components",
      executable="component_container",
      composable_node_descriptions=[
        ComposableNode(
          package="moveit_hybrid_planning",
          plugin="moveit::hybrid_planning::GlobalPlannerComponent",
          name="global_planner",
          parameters=[
            common_hybrid_planning_param,
            global_planner_param,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning_cartesian_limits,
            moveit_controllers,
          ],
        ),
        ComposableNode(
          package="moveit_hybrid_planning",
          plugin="moveit::hybrid_planning::LocalPlannerComponent",
          name="local_planner",
          parameters=[
            common_hybrid_planning_param,
            local_planner_param,
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            servo_params,
            moveit_controllers,
          ],
        ),
        ComposableNode(
          package="moveit_hybrid_planning",
          plugin="moveit::hybrid_planning::HybridPlanningManager",
          name="hybrid_planning_manager",
          parameters=[
            common_hybrid_planning_param,
            hybrid_planning_manager_param,
          ],
        ),
      ],
      output="screen",
    )


    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('ramsai_description'), 'rviz', 'ramsai.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            robot_description_kinematics,
            planning_pipelines_config,
            ompl_planning_config,
        ],
        condition=IfCondition(start_rviz),
    )

    nodes = [
        move_group_node,
        planning_container,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)

# Copyright 2023 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Maciej Bednarczyk

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile


def generate_launch_description():

  # Get URDF via xacro
  robot_description_content = Command(
    [
      PathJoinSubstitution([FindExecutable(name='xacro')]),
      ' ',
      PathJoinSubstitution(
        [FindPackageShare('ramsai_description'), 'config', 'ramsai.config.xacro']
      ),
    ]
  )

  robot_description = {'robot_description': robot_description_content}

  # Get SRDF via xacro
  robot_description_semantic_content = Command(
    [
      PathJoinSubstitution([FindExecutable(name='xacro')]),
      ' ',
      PathJoinSubstitution(
        [FindPackageShare('ramsai_description'), 'srdf', 'iiwa.srdf.xacro']
      ),
      ' ',
      'name:=',
      'iiwa_print',
    ]
  )

  robot_description_semantic = {
    'robot_description_semantic': robot_description_semantic_content
  }

  kinematics_yaml = PathJoinSubstitution(
    [FindPackageShare('iiwa_description'), 'moveit2', 'kinematics.yaml']
  )

  joint_limits_yaml = PathJoinSubstitution(
    [FindPackageShare('iiwa_description'), 'moveit2', 'iiwa_joint_limits.yaml']
  )

  cartesian_limits_yaml = PathJoinSubstitution(
    [FindPackageShare('iiwa_description'), 'moveit2', 'iiwa_cartesian_limits.yaml']
  )

  robot_cartesian_config = ParameterFile(
    PathJoinSubstitution(
      [FindPackageShare('ramsai_planning'), 'config', 'robot_cartesian_config.yaml']
    ),
    allow_substs=True
  )

  common_hybrid_planning_param = PathJoinSubstitution(
    [FindPackageShare('ramsai_description'), 'moveit2', 'common_hybrid_planning_params.yaml']
  )


  hybrid_node = Node(
    name = 'robot_cartesian_hybrid_planning',
    package ='ramsai_planning',
    executable ='robot_cartesian_hybrid_planning',
    output ='screen',
    parameters = [
      robot_description,
      robot_description_semantic,
      common_hybrid_planning_param,
      kinematics_yaml,
      joint_limits_yaml,
      cartesian_limits_yaml,
      robot_cartesian_config,
    ],
  )

  return LaunchDescription([hybrid_node])
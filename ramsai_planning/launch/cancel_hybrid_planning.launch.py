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


  common_hybrid_planning_param = PathJoinSubstitution(
    [FindPackageShare('ramsai_description'), 'moveit2', 'common_hybrid_planning_params.yaml']
  )


  cancel_hybrid_node = Node(
    name = 'cancel_hybrid_planning',
    package ='ramsai_planning',
    executable ='cancel_hybrid_planning',
    output ='screen',
    parameters = [
      common_hybrid_planning_param,
    ]

  )

  return LaunchDescription([cancel_hybrid_node])

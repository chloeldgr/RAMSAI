#!/usr/bin/env python3
# Copyright 2023 ICube Laboratory, University of Strasbourg
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
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='printer3d_manager',
            namespace='printer1',
            executable='printer_control_node',
            name='printer_control_node'
        ),
        Node(
            package='printer3d_image_capture',
            namespace='printer1',
            executable='image_capture_node',
            name='image_capture_node'
        ),
        Node(
            package='printer3d_profile_capture',
            namespace='printer1',
            executable='gocatorSensorNode',
            name='profile_capture_node'
        )
    ])

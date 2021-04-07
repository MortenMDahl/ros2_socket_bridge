#!/usr/bin/env python3

# Copyright 2021 Morten Melby Dahl.
# Copyright 2021 Norwegian University of Science and Technology.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
import sys
from rclpy.utilities import remove_ros_args
import argparse

def generate_launch_description(argv=sys.argv[1:]):

    # Can be anything to fit your desired method of naming. Must match with server.
    name = 'robot3'

    # Whether or not to use robot name as a prefix for incoming topics
    use_name = 'false'

    # Any 32 url-safe base64-encoded bytes passed in as a string.
    # This can be generated using the generate_key.py script in the main folder.
    # Must match with server key.
    encryption_key = 'VdzT2kwMacThZWkBigjbtte9iRjW8djEJ10JiemVwLM='

    spesific_robot_dir = LaunchConfiguration(
        'spesific_robot_dir',
        default=os.path.join(
            get_package_share_directory('rdb_client'),
            'config',
            'multi_robot',
            'robot3.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'spesific_robot_dir',
            default_value=spesific_robot_dir,
            description='Full path to parameter file to load'),

        launch_ros.actions.Node(
            package="rdb_client",
            executable="client",
            name="client_node",
            output="screen",
            emulate_tty=True,
            arguments=['-name', name, '-key', encryption_key, '-usename', use_name],
            parameters=[spesific_robot_dir]),
 ])
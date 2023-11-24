#!/usr/bin/env python
# -*- coding:utf-8 -*-

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
#
import pathlib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    pkg_share = pathlib.Path(FindPackageShare('sync_parameter_server').find('sync_parameter_server'))
    args = []
    args.append(DeclareLaunchArgument('config_file', default_value=str(pkg_share / 'config/sample.yaml')))

    node = Node(package='sync_parameter_server',
                executable='sync_parameter_server',
                name='sync_parameter_server',
                output='both',
                parameters=[LaunchConfiguration('config_file')])

    return LaunchDescription(args + [
        node,
    ])

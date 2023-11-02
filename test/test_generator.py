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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

import launch_generator


def generate_sample_launch_description() -> LaunchDescription:
    """Generate general sample launch description.

    Returns:
        Launch description.
    """
    background_r_launch_arg = DeclareLaunchArgument('background_r', default_value=TextSubstitution(text='0'))
    turtlesim_world_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('turtlesim'), 'launch'), '/multisim.launch.py']))

    config = os.path.join(get_package_share_directory('turtlesim'), 'config', 'turtlesim.yaml')

    node = Node(package='turtlesim',
                executable='turtlesim_node',
                name='sim',
                parameters=[config, {
                    'background_r': LaunchConfiguration('background_r'),
                }])

    turtlesim_world_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('turtlesim'), 'launch'), '/multisim.launch.py']))
    turtlesim_world_2_with_namespace = GroupAction(actions=[
        PushRosNamespace('turtlesim2'),
        turtlesim_world_2,
    ])

    return LaunchDescription([background_r_launch_arg, turtlesim_world_1, node, turtlesim_world_2_with_namespace])


def sample_generator() -> LaunchDescription:
    """Generate sample launch description with launch generator.

    Returns:
        Launch description.
    """
    gen = launch_generator.Generator()
    background_r = gen.add_argument('background_r', default_value='0')
    gen.add_include_launch_description('turtlesim', 'launch/multisim.launch.py')
    config = os.path.join(get_package_share_directory('turtlesim'), 'config', 'turtlesim.yaml')
    gen.add_node('turtlesim_node', 'turtlesim', 'sim', parameters=[config, {'background_r': background_r}])
    group = gen.add_group('turtlesim2')
    group.add_include_launch_description('turtlesim', 'launch/multisim.launch.py')
    return LaunchDescription(gen.generate_launch_description())


def test_generator() -> None:
    """Test generator."""
    generator = launch_generator.Generator()
    assert generator.launch_description == []
    assert generator.generate_launch_description() == []
    # TODO: Compare launch description
    # assert generate_sample_launch_description() == sample_generator()

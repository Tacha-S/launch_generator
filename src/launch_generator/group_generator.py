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
import typing

import launch
import launch_ros

import launch_generator


class GroupGenerator(launch_generator.BaseGenerator):
    """Group generator."""

    def __init__(self, namespace: launch.some_substitutions_type.SomeSubstitutionsType | None = None) -> None:
        """Initialize.

        Args:
            namespace: Group namespace. Defaults to None.
        """
        super().__init__()
        if namespace is not None:
            self.launch_description.append(launch_ros.actions.PushRosNamespace(namespace))

    def generate_launch_description(self) -> list[launch.action.Action]:
        """Generate launch description.

        Returns:
            Launch description.
        """
        return launch.actions.GroupAction(actions=[
            action if not isinstance(action, launch_generator.BaseGenerator) else action.generate_launch_description()
            for action in self.launch_description
        ])

    def add_argument(self,
                     name: typing.Text,
                     default_value: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
                     description: typing.Text | None = None,
                     choices: typing.Iterable[typing.Text] | None = None) -> launch.actions.DeclareLaunchArgument:
        """Declare launch argument and get it.

        Returns:
            Launch configuration.
        """
        arg = launch.actions.DeclareLaunchArgument(name,
                                                   default_value=default_value,
                                                   description=description,
                                                   choices=choices)
        self.launch_description.append(arg)
        return launch.substitutions.LaunchConfiguration(name)

    def add_node(self,
                 executable: launch.some_substitutions_type.SomeSubstitutionsType,
                 package: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
                 name: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
                 namespace: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
                 exec_name: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
                 parameters: launch_ros.parameters_type.SomeParameters | None = None,
                 remappings: launch_ros.remap_rule_type.SomeRemapRules | None = None,
                 ros_arguments: typing.Iterable[launch.some_substitutions_type.SomeSubstitutionsType] | None = None,
                 arguments: typing.Iterable[launch.some_substitutions_type.SomeSubstitutionsType] | None = None,
                 **kwargs) -> launch_ros.actions.Node:
        node = launch_ros.actions.Node(executable=executable,
                                       package=package,
                                       name=name,
                                       namespace=namespace,
                                       exec_name=exec_name,
                                       parameters=parameters,
                                       remappings=remappings,
                                       ros_arguments=ros_arguments,
                                       arguments=arguments,
                                       **kwargs)
        self.launch_description.append(node)
        return node

    def add_include_launch_description(self,
                                       package: launch.some_substitutions_type.SomeSubstitutionsType,
                                       relative_file_path: typing.Text,
                                       launch_arguments: typing.Iterable[
                                           tuple[launch.some_substitutions_type.SomeSubstitutionsType,
                                                 launch.some_substitutions_type.SomeSubstitutionsType]] = None,
                                       **kwargs) -> launch.actions.IncludeLaunchDescription:
        """Include launch description.

        Args:
            package: Package name.
            relative_file_path: Relative file path.
            launch_arguments: Launch arguments. Defaults to None.

        Raises:
            FileNotFoundError: Include launch file not found.

        Returns:
            Include launch description.
        """
        include_file = pathlib.Path(
            launch_ros.substitutions.FindPackageShare(package).find(package)) / relative_file_path
        if not include_file.exists():
            raise FileNotFoundError(f"Include launch file not found: {include_file}")
        include = launch.actions.IncludeLaunchDescription(str(include_file),
                                                          launch_arguments=launch_arguments,
                                                          **kwargs)
        self.launch_description.append(include)
        return include

    def add_group(
        self,
        namespace: launch.some_substitutions_type.SomeSubstitutionsType | None = None
    ) -> typing.TypeVar("TGroupGenerator", bound="GroupGenerator"):
        """Add group.

        Args:
            namespace: Group namespace. Defaults to None.

        Returns:
            Group generator.
        """
        group = launch_generator.GroupGenerator(namespace=namespace)
        self.launch_description.append(group)
        return group

    def add_container(
        self,
        name: launch.some_substitutions_type.SomeSubstitutionsType,
        namespace: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
        composable_node_descriptions: list[launch_ros.descriptions.ComposableNode] = []
    ) -> launch_generator.ContainerGenerator:
        """Add composable node container.

        Args:
            name: Container name.
            namespace: Container namespace. Defaults to None.
            composable_node_descriptions: Composable node descriptions. Defaults to [].

        Returns:
            Container generator.
        """
        gen = launch_generator.ContainerGenerator(name=name,
                                                  namespace=namespace,
                                                  composable_node_descriptions=composable_node_descriptions)
        self.launch_description.append(gen)
        return gen

    def add_action(self, action: launch.action.Action) -> launch.action.Action:
        """Add any action.

        Args:
            action: Action.

        Returns:
            Action.
        """
        self.launch_description.append(action)
        return action

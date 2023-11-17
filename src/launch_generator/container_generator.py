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

import launch
import launch_ros

from launch_generator.base_generator import BaseGenerator


class ContainerGenerator(BaseGenerator):
    """Generate launch description for composable node container."""

    def __init__(self,
                 name: launch.some_substitutions_type.SomeSubstitutionsType,
                 namespace: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
                 composable_node_descriptions: list[launch_ros.descriptions.ComposableNode] = [],
                 **kwargs) -> None:
        """Initialize.

        Args:
            name: Composable node container name.
            namespace: Composable node container namespace. Defaults to None.
            composable_node_descriptions: Composable node descriptions. Defaults to [].
        """
        super().__init__()
        self.__name = name
        self.__namespace = namespace
        self.launch_description += composable_node_descriptions
        self.__kwargs = kwargs

    def generate_launch_description(self) -> launch_ros.actions.ComposableNodeContainer:
        """Generate launch description.

        Returns:
            Launch description.
        """
        return launch_ros.actions.ComposableNodeContainer(
            package='rclcpp_components',
            namespace=self.__namespace if self.__namespace is not None else '',
            executable='component_container',
            name=self.__name,
            composable_node_descriptions=[
                action if not isinstance(action, BaseGenerator) else action.generate_launch_description()
                for action in self.launch_description
            ],
            **self.__kwargs,
        )

    def add_composable_node(self,
                            package: launch.some_substitutions_type.SomeSubstitutionsType,
                            plugin: launch.some_substitutions_type.SomeSubstitutionsType,
                            name: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
                            namespace: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
                            parameters: launch_ros.parameters_type.SomeParameters | None = None,
                            remappings: launch_ros.remap_rule_type.SomeRemapRules | None = None,
                            extra_arguments: launch_ros.parameters_type.SomeParameters | None = None,
                            condition: launch.Condition | None = None) -> launch_ros.descriptions.ComposableNode:
        """Add composable node to the container.

        Args:
            package: Package name.
            plugin: Plugin name.
            name: Node name. Defaults to None.
            namespace: Node namespace. Defaults to None.
            parameters: Node parameters. Defaults to None.
            remappings: Remappings. Defaults to None.
            extra_arguments: Extra arguments. Defaults to None.
            condition: Condition. Defaults to None.

        Returns:
            Composable node.
        """
        node = launch_ros.descriptions.ComposableNode(package=package,
                                                      plugin=plugin,
                                                      name=name,
                                                      namespace=namespace,
                                                      parameters=parameters,
                                                      remappings=remappings,
                                                      extra_arguments=extra_arguments,
                                                      condition=condition)
        self.launch_description.append(node)
        return node

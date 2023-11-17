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

from __future__ import annotations

import pathlib
import typing

import launch
import launch_ros

from launch_generator.base_generator import BaseGenerator
from launch_generator.container_generator import ContainerGenerator


class CommonGenerator(BaseGenerator):
    """Group generator."""
    event_action_type = (launch.some_actions_type.SomeActionsType
                         | typing.Callable[
                             [launch.events.process.ProcessExited, launch.launch_context.LaunchContext],
                             launch.some_actions_type.SomeActionsType | None,
                         ]
                         | None)
    event_io_type = (typing.Callable[[launch.events.process.ProcessIO],
                                     launch.some_actions_type.SomeActionsType | None] | None)

    def __init__(self) -> None:
        """Initialize.

        Args:
            namespace: Group namespace. Defaults to None.
        """
        super().__init__()

    def generate_launch_description(self) -> list[launch.action.Action]:
        """Generate launch description.

        Raises:
            NotImplementedError: Not implemented.

        Returns:
            Launch description.
        """
        raise NotImplementedError()

    def add_argument(
        self,
        name: typing.Text,
        default_value: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
        description: typing.Text | None = None,
        choices: typing.Iterable[typing.Text] | None = None,
    ) -> launch.substitutions.LaunchConfiguration:
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

    def add_node(
        self,
        executable: launch.some_substitutions_type.SomeSubstitutionsType,
        package: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
        name: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
        namespace: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
        exec_name: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
        parameters: launch_ros.parameters_type.SomeParameters | None = None,
        remappings: launch_ros.remap_rule_type.SomeRemapRules | None = None,
        ros_arguments: typing.Iterable[launch.some_substitutions_type.SomeSubstitutionsType] | None = None,
        arguments: typing.Iterable[launch.some_substitutions_type.SomeSubstitutionsType] | None = None,
        **kwargs,
    ) -> launch_ros.actions.Node:
        """Add node.

        Args:
            executable: Executable entry point name.
            package: Package name.
            name: Node name. Defaults to None.
            namespace: Node namespace. Defaults to None.
            exec_name: The label used to represent the process. Defaults to None.
            parameters: Node parameters. Defaults to None.
            remappings: Node remappings. Defaults to None.
            ros_arguments: ROS arguments. Defaults to None.
            arguments: Arguments. Defaults to None.

        Returns:
            Node description.
        """
        node = launch_ros.actions.Node(
            executable=executable,
            package=package,
            name=name,
            namespace=namespace,
            exec_name=exec_name,
            parameters=parameters,
            remappings=remappings,
            ros_arguments=ros_arguments,
            arguments=arguments,
            **kwargs,
        )
        self.launch_description.append(node)
        return node

    def add_include_launch_description(
        self,
        package: launch.some_substitutions_type.SomeSubstitutionsType,
        relative_file_path: typing.Text,
        launch_arguments: typing.Iterable[tuple[
            launch.some_substitutions_type.SomeSubstitutionsType,
            launch.some_substitutions_type.SomeSubstitutionsType,
        ]] = None,
        **kwargs,
    ) -> launch.actions.IncludeLaunchDescription:
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
        include_file = (pathlib.Path(launch_ros.substitutions.FindPackageShare(package).find(package))
                        / relative_file_path)
        if not include_file.exists():
            raise FileNotFoundError(f'Include launch file not found: {include_file}')
        include = launch.actions.IncludeLaunchDescription(str(include_file),
                                                          launch_arguments=launch_arguments,
                                                          **kwargs)
        self.launch_description.append(include)
        return include

    def add_group(self,
                  namespace: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
                  **kwargs) -> CommonGenerator:
        """Add group.

        Args:
            namespace: Group namespace. Defaults to None.

        Returns:
            Group generator.
        """
        from .group_generator import GroupGenerator
        group = GroupGenerator(namespace=namespace, **kwargs)
        self.launch_description.append(group)
        return group

    def add_container(
        self,
        name: launch.some_substitutions_type.SomeSubstitutionsType,
        namespace: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
        composable_node_descriptions: list[launch_ros.descriptions.ComposableNode] = [],
        **kwargs,
    ) -> ContainerGenerator:
        """Add composable node container.

        Args:
            name: Container name.
            namespace: Container namespace. Defaults to None.
            composable_node_descriptions: Composable node descriptions. Defaults to [].

        Returns:
            Container generator.
        """
        gen = ContainerGenerator(name=name,
                                 namespace=namespace,
                                 composable_node_descriptions=composable_node_descriptions,
                                 **kwargs)
        self.launch_description.append(gen)
        return gen

    def add_load_composable_nodes(self, target_container: launch.some_substitutions_type.SomeSubstitutionsType
                                  | launch_ros.actions.ComposableNodeContainer,
                                  composable_node_descriptions: list[launch_ros.descriptions.ComposableNode],
                                  **kwargs) -> launch_ros.actions.LoadComposableNodes:
        """Add load composable nodes.

        Args:
            target_container: Target container.
            composable_node_descriptions: Composable node descriptions.

        Returns:
            Load composable nodes action.
        """
        load_composable_nodes = launch_ros.actions.LoadComposableNodes(
            composable_node_descriptions=composable_node_descriptions, target_container=target_container, **kwargs)
        self.launch_description.append(load_composable_nodes)
        return load_composable_nodes

    def add_register_event_handler(
        self,
        target_action: typing.Callable[[launch.actions.ExecuteLocal], bool]
        | launch.actions.ExecuteLocal,
        trigger_type: str,
        **kwargs,
    ) -> CommonGenerator:
        """Add register event handler.

        Args:
            target_action: Target action.
            trigger_type: Event trigger type. EventTriggerType.ON_EXIT, ON_COMPLETION, ...
                          or str (e.g. 'on_exit') are available.

        Returns:
            Event handler generator.
        """
        from .event_handler_generator import EventHandlerGenerator
        gen = EventHandlerGenerator(target_action=target_action, trigger_type=trigger_type, **kwargs)
        self.launch_description.append(gen)
        return gen

    def add_ros_timer(self, period: float | launch.some_substitutions_type.SomeSubstitutionsType,
                      **kwargs) -> CommonGenerator:
        """Add actions to be executed after a period of time.

        Args:
            period: Time period.

        Returns:
            Ros timer generator.
        """
        from .timer_generator import TimerGenerator
        gen = TimerGenerator(period=period, **kwargs)
        self.launch_description.append(gen)
        return gen

    def add_set_parameter(self, name: launch.some_substitutions_type.SomeSubstitutionsType,
                          value: launch_ros.parameters_type.SomeParameterValue,
                          **kwargs) -> launch_ros.actions.SetParameter:
        """Add set parameter.

        Args:
            name: Parameter name.
            value: Parameter value.

        Returns:
            Set parameter action.
        """
        set_parameter = launch_ros.actions.SetParameter(name, value, **kwargs)
        self.launch_description.append(set_parameter)
        return set_parameter

    def add_set_parameters_from_file(self, package: launch.some_substitutions_type.SomeSubstitutionsType,
                                     relative_file_path: typing.Text,
                                     **kwargs) -> launch_ros.actions.SetParametersFromFile:
        """Add set parameters from file.

        Args:
            package: Package name.
            relative_file_path: Relative file path.

        Raises:
            FileNotFoundError: Parameter file not found.

        Returns:
            Set parameters from file action.
        """
        param_file = (pathlib.Path(launch_ros.substitutions.FindPackageShare(package).find(package))
                      / relative_file_path)
        if not param_file.exists():
            raise FileNotFoundError(f'Parameter file not found: {param_file}')
        set_parameters_from_file = launch_ros.actions.SetParametersFromFile(str(param_file), **kwargs)
        self.launch_description.append(set_parameters_from_file)
        return set_parameters_from_file

    def add_set_remap(self, src: launch.some_substitutions_type.SomeSubstitutionsType,
                      dst: launch.some_substitutions_type.SomeSubstitutionsType,
                      **kwargs) -> launch_ros.actions.SetRemap:
        """Add set remap.

        Args:
            src: Source topic name.
            dst: Destination topic name.

        Returns:
            Set remap action.
        """
        set_remap = launch_ros.actions.SetRemap(src, dst, **kwargs)
        self.launch_description.append(set_remap)
        return set_remap

    def add_set_environment_variable(self, name: launch.some_substitutions_type.SomeSubstitutionsType,
                                     value: launch.some_substitutions_type.SomeSubstitutionsType,
                                     **kwargs) -> launch.actions.SetEnvironmentVariable:
        """Add set environment variable.

        Args:
            name: Env name.
            value: Env value.

        Returns:
            Set environment variable action.
        """
        set_env_var = launch.actions.SetEnvironmentVariable(name, value, **kwargs)
        self.launch_description.append(set_env_var)
        return set_env_var

    def add_action(self, action: launch.action.Action) -> launch.action.Action:
        """Add any action.

        Args:
            action: Action.

        Returns:
            Action.
        """
        self.launch_description.append(action)
        return action

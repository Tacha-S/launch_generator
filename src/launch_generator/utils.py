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
import re
import typing

import launch
import launch_ros
import yaml

from launch_generator.common_generator import CommonGenerator


def condition(
    expression: str | launch.substitution.Substitution | typing.Iterable,
    reverse: bool = False,
) -> launch.conditions.IfCondition:
    """Generate condition.

    Args:
        expression: Condition expression.
        reverse: Reverse condition. Defaults to False.

    Returns:
        IfCondition if reverse is False, otherwise UnlessCondition.
    """

    def __flexible_bool(exp: launch.substitution.Substitution) -> list:
        return [
            '( True if "',
            exp,
            '" == "true" else ( False if "',
            exp,
            '" == "false" else ',
            exp,
            '))',
        ]

    condition_type = launch.conditions.UnlessCondition if reverse else launch.conditions.IfCondition
    if not isinstance(expression, str):
        extend_expression = []
        if isinstance(expression, launch.substitution.Substitution):
            extend_expression = __flexible_bool(expression)
        else:
            for exp in expression:
                if isinstance(exp, launch.substitution.Substitution):
                    extend_expression += __flexible_bool(exp)
                else:
                    extend_expression.append(exp)
        return condition_type(launch.substitutions.PythonExpression(extend_expression))
    return condition_type(launch.substitutions.PythonExpression(expression))


def package_path(package_name: str) -> pathlib.Path:
    """Get package path.

    Args:
        package_name: Package name.

    Returns:
        Package path.
    """
    return pathlib.Path(launch_ros.substitutions.FindPackageShare(package_name).find(package_name))


def load_param_file(package: launch.some_substitutions_type.SomeSubstitutionsType,
                    relative_file_path: typing.Text) -> dict:
    """Load param file to dict. Replace $(arg xxx) with LaunchConfiguration(xxx).

    Args:
        package: Package name.
        relative_file_path: Relative file path.

    Returns:
        Parameters.
    """
    path = package_path(package) / relative_file_path
    with open(path, 'r') as f:
        parameters = yaml.safe_load(f.read())

    def __replace_arg(params: dict[str, typing.Any]) -> None:
        """Replace $(arg xxx) with LaunchConfiguration(xxx).

        Args:
            params: Parameters.
        """
        regex = re.compile(r'\$\(arg \w+?\)')
        replaced_keys = {}
        for key, value in params.items():
            if isinstance(value, dict):
                __replace_arg(value)
            elif isinstance(value, str):
                match = regex.findall(value)
                if match:
                    replaced = []
                    start = 0
                    for m in match:
                        i = value.find(m)
                        if i > start:
                            replaced.append(value[start:i])
                        replaced.append(launch.substitutions.LaunchConfiguration(m[6:-1]))
                        start = i + len(m)
                    if start < len(value):
                        replaced.append(value[start:])
                    params[key] = replaced
                    value = replaced

            match = regex.findall(key)
            if match:
                replaced = []
                start = 0
                for m in match:
                    i = key.find(m)
                    if i > start:
                        replaced.append(key[start:i])
                    replaced.append(launch.substitutions.LaunchConfiguration(m[6:-1]))
                    start = i + len(m)
                if start < len(key):
                    replaced.append(key[start:])
                replaced = tuple(replaced)
                replaced_keys[replaced] = value
        params.update(replaced_keys)

    __replace_arg(parameters)

    return parameters


def set_action_remap(gen: CommonGenerator, src: launch.some_substitutions_type.SomeSubstitutionsType,
                     dst: launch.some_substitutions_type.SomeSubstitutionsType) -> None:
    """Remap action namespace from src to dst.

    Following topic and service will be remapped:
    - {src}/_action/send_goal
    - {src}/_action/cancel_goal
    - {src}/_action/get_result
    - {src}/_action/feedback
    - {src}/_action/status

    Args:
        gen: Launch generator.
        src: Source action namespace.
        dst: Destination action namespace.
    """
    action_services = ['send_goal', 'cancel_goal', 'get_result']
    action_topics = ['feedback', 'status']
    for srv in action_services:
        gen.add_set_remap([src, f'/_action/{srv}'], [dst, f'/_action/{srv}'])
    for topic in action_topics:
        gen.add_set_remap([src, f'/_action/{topic}'], [dst, f'/_action/{topic}'])

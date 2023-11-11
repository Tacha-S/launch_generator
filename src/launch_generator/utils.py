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

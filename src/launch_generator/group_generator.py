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
from launch_generator.common_generator import CommonGenerator


class GroupGenerator(CommonGenerator):
    """Group generator."""

    def __init__(self,
                 namespace: launch.some_substitutions_type.SomeSubstitutionsType | None = None,
                 **kwargs) -> None:
        """Initialize.

        Args:
            namespace: Group namespace. Defaults to None.
        """
        super().__init__()
        if namespace is not None:
            self.launch_description.append(launch_ros.actions.PushRosNamespace(namespace))
        self.__kwargs = kwargs

    def generate_launch_description(self) -> launch.actions.GroupAction:
        """Generate launch description.

        Returns:
            Launch description.
        """
        return launch.actions.GroupAction(actions=[
            action if not isinstance(action, BaseGenerator) else action.generate_launch_description()
            for action in self.launch_description
        ],
                                          **self.__kwargs)

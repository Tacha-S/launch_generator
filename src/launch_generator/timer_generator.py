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


class TimerGenerator(CommonGenerator):
    """Generate launch description for RosTimer."""

    def __init__(self, period: float | launch.some_substitutions_type.SomeSubstitutionsType, **kwargs) -> None:
        """Initialize.

        Args:
            namespace: Group namespace. Defaults to None.
        """
        super().__init__()
        self.__period = period
        self.__kwargs = kwargs

    def generate_launch_description(self) -> launch_ros.actions.RosTimer:
        """Generate launch description.

        Returns:
            Launch description.
        """
        return launch_ros.actions.RosTimer(
            period=self.__period,
            actions=[
                action if not isinstance(action, BaseGenerator) else action.generate_launch_description()
                for action in self.launch_description
            ],
            **self.__kwargs)

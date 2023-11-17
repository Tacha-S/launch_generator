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

import typing
from enum import Enum

import launch

from launch_generator.base_generator import BaseGenerator
from launch_generator.common_generator import CommonGenerator


class EventTriggerType(Enum):
    """Event trigger type."""
    ON_EXIT = 'on_exit'
    ON_COMPLETION = 'on_completion'
    ON_START = 'on_start'
    ON_SHUTDOWN = 'on_shutdown'
    ON_STDIN = 'on_stdin'
    ON_STDOUT = 'on_stdout'
    ON_STDERR = 'on_stderr'


class EventHandlerGenerator(CommonGenerator):
    """Generate launch description for register event handler."""

    def __init__(self, target_action: typing.Callable[[launch.actions.ExecuteLocal], bool]
                 | launch.actions.ExecuteLocal, trigger_type: str | EventTriggerType, **kwargs) -> None:
        """Initialize.

        Args:
            target_action: Target action.
            trigger_type: Event trigger type.
        """
        super().__init__()
        self.__target_action = target_action
        self.__trigger_type = trigger_type if isinstance(trigger_type,
                                                         EventTriggerType) else EventTriggerType(trigger_type)
        self.__kwargs = kwargs

    def generate_launch_description(self) -> launch.actions.RegisterEventHandler:
        """Generate launch description.

        Returns:
            Launch description.
        """
        match self.__trigger_type:
            case EventTriggerType.ON_EXIT:
                event_handler = launch.event_handlers.OnProcessExit(
                    target_action=self.__target_action,
                    on_exit=[
                        action if not isinstance(action, BaseGenerator) else action.generate_launch_description()
                        for action in self.launch_description
                    ])
            case EventTriggerType.ON_COMPLETION:
                event_handler = launch.event_handlers.OnExecutionComplete(
                    target_action=self.__target_action,
                    on_completion=[
                        action if not isinstance(action, BaseGenerator) else action.generate_launch_description()
                        for action in self.launch_description
                    ])
            case EventTriggerType.ON_START:
                event_handler = launch.event_handlers.OnProcessStart(
                    target_action=self.__target_action,
                    on_start=[
                        action if not isinstance(action, BaseGenerator) else action.generate_launch_description()
                        for action in self.launch_description
                    ])
            case EventTriggerType.ON_SHUTDOWN:
                event_handler = launch.event_handlers.OnShutdown(
                    target_action=self.__target_action,
                    on_shutdown=[
                        action if not isinstance(action, BaseGenerator) else action.generate_launch_description()
                        for action in self.launch_description
                    ])
            case EventTriggerType.ON_STDIN:
                event_handler = launch.event_handlers.OnProcessIO(
                    target_action=self.__target_action,
                    on_stdin=[
                        action if not isinstance(action, BaseGenerator) else action.generate_launch_description()
                        for action in self.launch_description
                    ])
            case EventTriggerType.ON_STDOUT:
                event_handler = launch.event_handlers.OnProcessIO(
                    target_action=self.__target_action,
                    on_stdout=[
                        action if not isinstance(action, BaseGenerator) else action.generate_launch_description()
                        for action in self.launch_description
                    ])
            case EventTriggerType.ON_STDERR:
                event_handler = launch.event_handlers.OnProcessIO(
                    target_action=self.__target_action,
                    on_stderr=[
                        action if not isinstance(action, BaseGenerator) else action.generate_launch_description()
                        for action in self.launch_description
                    ])
            case _:
                raise ValueError(f'Unknown trigger type: {self.__trigger_type}')

        return launch.actions.RegisterEventHandler(event_handler, **self.__kwargs)

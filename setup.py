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

import setuptools

setuptools.setup(
    name='launch_generator',
    version='0.4.0',
    author='Tatsuro Sakaguchi',
    author_email='tatsuro.sakaguchi@g.softbank.co.jp',
    description='The package `launch-generator` is a tool to easily generate launch descriptions for ROS 2.',
    url='https://github.com/Tacha-S/launch_generator',
    package_dir={'': 'src'},
    packages=setuptools.find_packages('src', include=['launch_generator']),
    install_requires=['pyyaml'],
)

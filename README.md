# launch-generator

The package `launch-generator` is a tool to easily generate launch descriptions for ROS 2.
It is based on the `launch` and `launch_ros` packages.

## Installation

```bash
pip install launch-generator
```

## Usage

`Generator` is the main class of the package. It is used to generate launch descriptions.
It can add some actions with the `add_*` methods and generate the launch description with the `generate_launch_description` method.
`add_*` methods arguments are the same as the original basically.
`add_group()` and `add_container()` methods return a generator object that can be used to add internal actions to the group or container.

```python
from launch import LaunchDescription
import launch_generator

def generate_launch_description() -> LaunchDescription:
    gen = launch_generator.Generator()

    # Add launch arguments
    arg_configuration = gen.add_arg('arg_name', 'default_value')

    # Add nodes and use the launch arguments
    node = gen.add_node('executable_name', 'package_name', name=arg_configuration)

    # Include other launch files
    include = gen.include_launch('package_name', 'launch/file/path.launch.py')

    # Add group
    group = gen.add_group('namespace')

    # Add nodes to group
    group.add_node('executable_name', 'package_name')

    return LaunchDescription(gen.generate_launch_description())
```

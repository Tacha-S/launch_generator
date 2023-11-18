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

## Migration

### imports

launch_ros

```python
from launch import LaunchDescription
from launch_ros.actions import *
from launch_ros.substitutions import *
...
```

launch_generator

```python
from launch import LaunchDescription
import launch_generator
```

### Argument

launch_ros

```python
# declare argument
arg = launch.actions.DeclareLaunchArgument(name,
                                           default_value=default_value,
                                           description=description,
                                           choices=choices)

# use argument
Node(
    ...
    parameters=[{'param_name': launch.substitutions.LaunchConfiguration(name)}]
)

# add argument to launch description
return LaunchDescription([arg, ...])
```

launch_generator

```python
arg = gen.add_arg(name, default_value, description, choices)

Node(
    ...
    parameters=[{'param_name': arg}]
)

return LaunchDescription(gen.generate_launch_description())
```

### Node

launch_ros

```python
Node(
    package=package,
    executable=executable,
    name=name,
    namespace=namespace,
    parameters=parameters,
    remappings=remappings,
    arguments=arguments,
    **kwargs
)

# add node to launch description
return LaunchDescription([node, ...])
```

launch_generator

```python
node = gen.add_node(executable, package, name, namespace, parameters=parameters, remappings=remappings, ros_arguments=arguments, **kwargs)

return LaunchDescription(gen.generate_launch_description())
```

### IncludeLaunchDescription

launch_ros

```python
package_path = FindPackageShare('package_name').find('package_name')
launch_file_path = os.path.join(package_path, 'launch', 'file_name.launch.py')
include = IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file_path),
)

# add include to launch description
return LaunchDescription([include, ...])
```

launch_generator

```python
gen.add_include_launch_description('package_name', 'launch/file_name.launch.py')

return LaunchDescription(gen.generate_launch_description())
```

### GroupAction

launch_ros

```python
group = GroupAction(
    actions=[
        PushRosNamespace(
            namespace=namespace,
        ),
        Node(
            ...
        ),
        Node(
            ...
        ),
    ]
)

# add group to launch description
return LaunchDescription([group, ...])
```

launch_generator

```python
group = gen.add_group(namespace)
group.add_node(...)
group.add_node(...)

return LaunchDescription(gen.generate_launch_description())
```

### Composable node container

launch_ros

```python
container = ComposableNodeContainer(
    name=name,
    namespace=namespace,
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(...),
        ComposableNode(...),
    ],
)

# add container to launch description
return LaunchDescription([container, ...])
```

launch_generator

```python
container = gen.add_container(name, namespace)
container.add_composable_node(...)
container.add_composable_node(...)

return LaunchDescription(gen.generate_launch_description())
```

### Composable node

launch_ros

```python
node = ComposableNode(
    package=package,
    plugin=plugin,
    name=name,
    namespace=namespace,
    parameters=parameters,
    remappings=remappings,
    extra_arguments=extra_arguments,
)

return LaunchDescription([node, ...])
```

launch_generator

```python
node = container.add_composable_node(package, plugin, name, namespace, parameters, remappings, extra_arguments)

return LaunchDescription(gen.generate_launch_description())
```

### Load composable nodes to existing container

launch_ros

```python
load_composable_nodes = LoadComposableNodes(
    target_container='target_container_name',
    composable_node_descriptions=[
        ComposableNode(...),
        ComposableNode(...),
    ],
)

return LaunchDescription([load_composable_nodes, ...])
```

launch_generator

```python
load_composable_nodes = gen.add_load_composable_nodes('target_container_name', [
    gen.add_composable_node(...),
    gen.add_composable_node(...),
])

return LaunchDescription(gen.generate_launch_description())
```

### Register Event Handler

launch_ros

```python
register_event_handler = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=target_node,
        on_exit=[other_node],
    ),
)

return LaunchDescription([register_event_handler, ...])
```

launch_generator

```python
on_exit = gen.add_register_event_handler(target_node, launch_generator.EventTriggerType.ON_EXIT)
on_exit.add_action(executable, package, name)

return LaunchDescription(gen.generate_launch_description())
```

### Ros timer

launch_ros

```python

timer = RosTimer(
    period=period,
    actions=[
        Node(
            ...
        ),
        Node(
            ...
        ),
    ]
)

return LaunchDescription([timer, ...])
```

launch_generator

```python
timer = gen.add_ros_timer(period)
timer.add_node(...)
timer.add_node(...)

return LaunchDescription(gen.generate_launch_description())
```

### Set parameter

launch_ros

```python
set_parameter = SetParameter(
    name=name,
    value=value,
)

return LaunchDescription([set_parameter, ...])
```

launch_generator

```python
gen.add_set_parameter(name, value)

return LaunchDescription(gen.generate_launch_description())
```

### Set parameter file

launch_ros

```python
package_path = FindPackageShare('package_name').find('package_name')
config_file_path = os.path.join(package_path, 'config', 'file_name.yaml')
set_parameter_file = SetParameterFile(config_file_path)

return LaunchDescription([set_parameter_file, ...])
```

launch_generator

```python
gen.add_set_parameter_file('package_name', 'config/file_name.yaml')

return LaunchDescription(gen.generate_launch_description())
```

### Set remap

launch_ros

```python
set_remap = SetRemap(
    src=src_topic,
    dst=dst_topic,
)

return LaunchDescription([set_remap, ...])
```

launch_generator

```python
gen.add_set_remap(src_topic, dst_topic)

return LaunchDescription(gen.generate_launch_description())
```

### Set environment variable

launch_ros

```python
set_env = SetEnvironmentVariable(
    name=name,
    value=value,
)

return LaunchDescription([set_env, ...])
```

launch_generator

```python
gen.add_set_environment_variable(name, value)

return LaunchDescription(gen.generate_launch_description())
```

### Other entities

`add_action()` supports all the entities of `launch` and `launch_ros` packages.

launch_ros

```python

ns = PushRosNamespace(
    namespace=namespace,
)

return LaunchDescription([ns, ...])
```

launch_generator

```python
gen.add_action(PushRosNamespace(namespace))

return LaunchDescription(gen.generate_launch_description())
```

## Utilities

### condition()

`condition()` is a function to create IfCondition or UnlessCondition object.

```python
arg = gen.add_arg(name, default_value, description, choices)

# basic usage
gen.add_node('executable_name', 'package_name',
             condition=launch_generator.condition(arg)) # the node is launched only if arg is 'True' or 'true'

# reverse condition
gen.add_node('executable_name', 'package_name',
             condition=launch_generator.condition(arg, reverse=True)) # the node is launched only if arg is 'False' or 'false'

# python expression can be used
gen.add_node('executable_name', 'package_name',
             condition=launch_generator.condition([arg, '< 10'])) # the node is launched only if arg is less than 10

arg2 = gen.add_arg(name, default_value, description, choices)
# multiple conditions can be used
gen.add_node('executable_name', 'package_name',
             condition=launch_generator.condition([arg, '< 10 or ', arg2])) # the node is launched only if arg is less than 10 or greater than 20
```

### load_param_file()

`load_param_file()` is a function to load a yaml file to parameter dictionary.
The function can replace the value of parameters written in the form of `$(arg arg_name)` like `subst_value` of ROS1 with `LaunchConfiguration(arg_name)`.

Example of yaml file:

```yaml
node_name:
    ros__parameters:
        param1: $(arg arg_name)
        param2:
            sub1: prefix_$(arg arg_name)
            sub2: $(arg arg_name)_suffix
            sub3: $(arg arg_name)_$(arg_name2)
```

```python
arg = gen.add_arg('arg_name', 'default_value')
arg2 = gen.add_arg('arg_name2', 'default_value')
param_dict = launch_generator.load_param_file('package_name', 'config/file_name.yaml')

# param_dict = {
#     'node_name': {
#         'ros__parameters': {
#             'param1': launch.substitutions.LaunchConfiguration('arg_name'),
#             'param2': {
#                 'sub1': ['prefix_', launch.substitutions.LaunchConfiguration('arg_name')],
#                 'sub2': [launch.substitutions.LaunchConfiguration('arg_name'), '_suffix'],
#                 'sub3': [launch.substitutions.LaunchConfiguration('arg_name'), '_', launch.substitutions.LaunchConfiguration('arg_name2')],
#             }
#         }
#     }
# }

gen.add_node('executable_name', 'package_name', parameters=[param_dict['node_name']['ros__parameters']])

return LaunchDescription(gen.generate_launch_description())
```

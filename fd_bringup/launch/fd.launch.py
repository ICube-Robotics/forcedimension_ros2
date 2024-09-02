# Copyright 2022 ICube Laboratory, University of Strasbourg
#
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_orientation = LaunchConfiguration('use_orientation')
    use_clutch = LaunchConfiguration('use_clutch')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('fd_description'),
                    'config',
                    'fd.config.xacro',
                ]
            ),
            ' use_fake_hardware:=', use_fake_hardware,
            ' use_orientation:=', use_orientation,
            ' use_clutch:=', use_clutch,
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    phi_controllers = PathJoinSubstitution(
        [
            FindPackageShare('fd_description'),
            'config',
            'fd_controllers.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='fd',
        output='screen',
        parameters=[robot_description],
    )

    # A node to publish world -> iiwa_base transform
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=[
            '0.0', '0.0', '0.0', '3.1416', '0.0', '0.0',
            'world',
            'fd_base'
        ],
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='fd',
        parameters=[robot_description, phi_controllers],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    # Load controllers
    load_controllers = []
    for controller in [
        'fd_controller',
        'joint_state_broadcaster',
        'fd_ee_broadcaster',
        'fd_inertia_broadcaster',
        'fd_clutch_broadcaster',
    ]:
        load_controllers += [
            Node(
                package='controller_manager',
                executable='spawner',
                namespace='fd',
                arguments=[
                    controller,
                    '--controller-manager',
                    '/fd/controller_manager',
                ],
            ),
        ]

    nodes = [
        controller_manager_node,
        node_robot_state_publisher,
        static_tf,
    ] + load_controllers
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake r2c hardware interfaces'),
        DeclareLaunchArgument(
            'use_orientation',
            default_value='false',
            description='Read angular positions.velocities'
                        + '(WARNING! RPY parameterization)'
        ),
        DeclareLaunchArgument(
            'use_clutch',
            default_value='false',
            description='Enable clutch (read pos/vel/force and write force)'),
        ] + nodes)

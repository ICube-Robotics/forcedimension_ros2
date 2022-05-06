from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("fd_description"),
                    "config",
                    "fd.config.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    phi_controllers = PathJoinSubstitution(
        [
            FindPackageShare("fd_description"),
            "config",
            "fd_controllers.yaml",
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="fd",
        output="screen",
        parameters=[robot_description],
    )

    # A node to publish world -> iiwa_base transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["-0.8", "0.0", "0.4", "3.1416", "0.0", "0.0", "world", "fd_base"],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="fd",
        parameters=[robot_description, phi_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

     # Load controllers
    load_controllers = []
    for controller in ["fd_controller", "joint_state_broadcaster", "ee_pose_broadcaster"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner --controller-manager /fd/controller_manager {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    nodes = [
        controller_manager_node,
        node_robot_state_publisher,
        static_tf,
    ] + load_controllers
    return LaunchDescription(nodes)


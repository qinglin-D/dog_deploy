from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    real_node = Node(
        package="real",
        executable="real",
        name="real_node",
        output="screen",
    )

    rl_controller_node = Node(
        package="rl_controller",
        executable="rl_controller",
        name="rl_controller_node",
        output="screen",
    )

    return LaunchDescription([
        joy_node,
        real_node,
        rl_controller_node,
    ])

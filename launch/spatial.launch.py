import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    node_adnav_driver = Node(
        package='ros2-driver',
        executable='adnav_driver',
        output='screen',
        arguments=['115200', '/dev/ttyUSB0']
    )


    # Launch!
    return LaunchDescription([
        node_adnav_driver
    ])

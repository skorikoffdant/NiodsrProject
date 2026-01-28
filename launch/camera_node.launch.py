#Updated by Maryna Iovitsa

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen'
        ),
        Node(
            package='camera_subscriber',
            executable='camera_node',
            output='screen'
        )
    ])


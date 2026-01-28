# Updated by Maryna Iovitsa

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # USB camera
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            output='screen'
        ),

        # Camera subscriber
        Node(
            package='camera_subscriber',
            executable='camera_node',
            output='screen'
        ),

        # Start URSim (UR5)
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ur_client_library',
                'start_ursim.sh',
                '-m', 'ur5'
            ],
            output='screen'
        ),

        # Start UR driver with RViz
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'ur_robot_driver',
                'ur_control.launch.py',
                'ur_type:=ur5',
                'robot_ip:=192.168.56.101',
                'launch_rviz:=true'
            ],
            output='screen'
        )
    ])

#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your .sdf world file
    world_file = os.path.expanduser('~/ros2_ws/src/guided_navigation/world/stereo_world.sdf')

    # Exporting the Roboflow API key
    export_roboflow_key = SetEnvironmentVariable('ROBOFLOW_API_KEY', 'aFkoLbgUAThELZEBkgQ5')

    return LaunchDescription([
        # Start the Ignition Gazebo simulation
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_file],
            output='screen'
        ),

        # Set the Roboflow API key environment variable
        export_roboflow_key,        

        # Bridges for each camera
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='left_camera_1',
            arguments=['/world/empty/model/stereo_camera/link/left_camera_link/sensor/left_camera/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_2_bridge',
            arguments=['/world/empty/model/stereo_camera/link/right_camera_link/sensor/right_camera/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),        
        Node(
            package='guided_navigation',
            executable='stereo_camera.py',
            name='StereoCamera',
            output='screen'
        ),
    ])

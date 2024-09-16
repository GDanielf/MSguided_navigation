#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your .sdf world file
    world_file = os.path.expanduser('~/ros2_ws/src/guided_navigation/world/camera_world.sdf')

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
            name='camera_1_bridge',
            arguments=['/world/empty/model/rgbd_camera/link/link/sensor/camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_2_bridge',
            arguments=['/world/empty/model/rgbd_camera_1/link/link_1/sensor/camera_sensor_1/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_3_bridge',
            arguments=['/world/empty/model/rgbd_camera_2/link/link_2/sensor/camera_sensor_2/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_4_bridge',
            arguments=['/world/empty/model/rgbd_camera_3/link/link_3/sensor/camera_sensor_3/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        Node(
            package='guided_navigation',
            executable='multi_camera.py',
            name='MultiCamera',
            output='screen'
        ),
    ])

#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # Nó fixo para a chave da API do Roboflow
    export_roboflow_key = SetEnvironmentVariable('ROBOFLOW_API_KEY', 'aFkoLbgUAThELZEBkgQ5')

    # Nós fixos
    static_nodes = [
        export_roboflow_key,      

        # Bridges fixos
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            output='screen'
        ),
        Node(package='ros_gz_bridge',
             executable='parameter_bridge',
             name='robot_real_pose',
             arguments=['/model/marble_husky_sensor_config_5/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V'],
             output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_simulation_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
            output='screen'
        ),                
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='husky_odometry_bridge',
            arguments=['/model/marble_husky_sensor_config_5/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
            output='screen'
        ),

        # Node de navegação
        Node(
            package='guided_navigation',
            executable='multi_camera.py',
            name='MultiCamera',
            output='screen'
        ),
    ]

    # Geração dinâmica dos nós de câmera
    dynamic_nodes = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'camera_{i}_bridge',
            arguments=[f'/world/empty/model/rgbd_camera_{i}/link/link_{i}/sensor/camera_sensor_{i}/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ) for i in range(35)  # Vai de 0 até 34
    ]

    # Combina os nós estáticos e dinâmicos
    return LaunchDescription(static_nodes + dynamic_nodes)

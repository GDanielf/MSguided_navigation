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
        # Set the Roboflow API key environment variable
        export_roboflow_key,      

        #bridges for robot /cmd_vel   
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_simulation_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
            output='screen'
        ),
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
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_5_bridge',
            arguments=['/world/empty/model/rgbd_camera_4/link/link_4/sensor/camera_sensor_4/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_6_bridge',
            arguments=['/world/empty/model/rgbd_camera_5/link/link_5/sensor/camera_sensor_5/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_7_bridge',
            arguments=['/world/empty/model/rgbd_camera_6/link/link_6/sensor/camera_sensor_6/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_8_bridge',
            arguments=['/world/empty/model/rgbd_camera_7/link/link_7/sensor/camera_sensor_7/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),     

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_9_bridge',
            arguments=['/world/empty/model/rgbd_camera_8/link/link_8/sensor/camera_sensor_8/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_10_bridge',
            arguments=['/world/empty/model/rgbd_camera_9/link/link_9/sensor/camera_sensor_9/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_11_bridge',
            arguments=['/world/empty/model/rgbd_camera_10/link/link_10/sensor/camera_sensor_10/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_12_bridge',
            arguments=['/world/empty/model/rgbd_camera_11/link/link_11/sensor/camera_sensor_11/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_13_bridge',
            arguments=['/world/empty/model/rgbd_camera_12/link/link_12/sensor/camera_sensor_12/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_14_bridge',
            arguments=['/world/empty/model/rgbd_camera_13/link/link_13/sensor/camera_sensor_13/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_15_bridge',
            arguments=['/world/empty/model/rgbd_camera_14/link/link_14/sensor/camera_sensor_14/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_16_bridge',
            arguments=['/world/empty/model/rgbd_camera_15/link/link_15/sensor/camera_sensor_15/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='husky_odometry_bridge',
            arguments=['/model/marble_husky_sensor_config_5/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
            output='screen' 
        ),

        Node(
            package='guided_navigation',
            executable='multi_camera.py',
            name='MultiCamera',
            output='screen'
        ),
    ])

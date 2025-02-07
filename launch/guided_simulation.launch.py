#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import xacro

def generate_launch_description():
    # Nó fixo para a chave da API do Roboflow
    export_roboflow_key = SetEnvironmentVariable('ROBOFLOW_API_KEY', 'aFkoLbgUAThELZEBkgQ5')

    world_path = os.path.join(
            get_package_share_directory('guided_navigation'),  
            'world',
            'camera_world.sdf'
        )

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    guided_navigation_path = os.path.join(
        get_package_share_directory('guided_navigation'))
    
    xacro_file = os.path.join(guided_navigation_path,
                              'urdf',
                              'camera_robot.xacro.urdf')
    
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'camera_robot',
                   '-allow_renaming', 'true'],
    )

    #camera0
    load_joint_state_broadcaster_0 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster_0'],
        output='screen'
    )

    load_joint_trajectory_controller_0 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller_0'],
        output='screen'
    )

    load_imu_sensor_broadcaster_0 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'imu_sensor_broadcaster_0'],
        output='screen'
    )

    #camera1
    load_joint_state_broadcaster_1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster_1'],
        output='screen'
    )

    load_joint_trajectory_controller_1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller_1'],
        output='screen'
    )

    load_imu_sensor_broadcaster_1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'imu_sensor_broadcaster_1'],
        output='screen'
    )
    #camera2
    load_joint_state_broadcaster_2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster_2'],
        output='screen'
    )

    load_joint_trajectory_controller_2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller_2'],
        output='screen'
    )

    load_imu_sensor_broadcaster_2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'imu_sensor_broadcaster_2'],
        output='screen'
    )

    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    bridge_robot_real_pose = Node(package='ros_gz_bridge',
            executable='parameter_bridge',
            name='robot_real_pose',
            arguments=['/model/marble_husky_sensor_config_5/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V'],
            output='screen',
            parameters=[{'use_sim_time': True}]
    )
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_simulation_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )    

    # Geração dinâmica dos nós de câmera
    bridge_camera_0 =  Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'camera_bridge_0',
        arguments=[f'/world/empty/model/camera_robot/link/head_link_0/sensor/camera_sensor_0/image@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    bridge_camera_1 =  Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'camera_bridge_1',
        arguments=[f'/world/empty/model/camera_robot/link/head_link_1/sensor/camera_sensor_1/image@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    bridge_camera_2 =  Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'camera_bridge_2',
        arguments=[f'/world/empty/model/camera_robot/link/head_link_2/sensor/camera_sensor_2/image@sensor_msgs/msg/Image@gz.msgs.Image'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Combina os nós estáticos e dinâmicos
    return LaunchDescription([        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                              'launch', 'ign_gazebo.launch.py')]),
                              launch_arguments=[('gz_args', f'-r -v 3 -s {world_path}')]),            
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_broadcaster_0, load_joint_state_broadcaster_1, load_joint_state_broadcaster_2],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster_0,
                on_exit=[load_joint_trajectory_controller_0],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller_0,
                on_exit=[load_imu_sensor_broadcaster_0],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster_1,
                on_exit=[load_joint_trajectory_controller_1],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller_1,
                on_exit=[load_imu_sensor_broadcaster_1],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster_2,
                on_exit=[load_joint_trajectory_controller_2],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller_2,
                on_exit=[load_imu_sensor_broadcaster_2],
            )
        ),
        bridge_cmd_vel,
        bridge_robot_real_pose,
        bridge_clock,
        bridge_camera_0,
        bridge_camera_1,
        bridge_camera_2,
        node_robot_state_publisher,
        ignition_spawn_entity,
        export_roboflow_key,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
            ])

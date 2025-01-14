import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'guided_navigation'
    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'camera_robot.urdf')

    # Positions for each robot
    robot_positions = {
        'robo_0': {'x': '-3.0', 'y': '-7.4687'},
        'robo_1': {'x': '4.0', 'y': '7.4687'},
        'robo_2': {'x': '-9.9830', 'y': '-1.2500'}
    }

    spawn_robots = []

    for robot_name, position in robot_positions.items():
        spawn_robots.append(
            ExecuteProcess(
                cmd=[
                    'ign', 'model', '--spawn-file', urdf_file,
                    '--model-name', robot_name,
                    '--x', position['x'], '--y', position['y'], '--z', '0'
                ],
                output='screen'
            )
        )

    # Bridge for camera image topics
    bridge_camera_topics = []
    for robot_name in robot_positions.keys():
        bridge_camera_topics.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{robot_name}_camera_bridge',
                parameters=[{'use_sim_time': True}],
                arguments=[
                    f'/model/{robot_name}/camera_link/camera_sensor/image@sensor_msgs/msg/Image@gz.msgs.Image'
                ]
            )
        )

    # Bridge for joint command topics
    bridge_joint_topics = []
    for robot_name in robot_positions.keys():
        bridge_joint_topics.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'{robot_name}_joint_bridge',
                parameters=[{'use_sim_time': True}],
                arguments=[
                    f'/model/{robot_name}/joint/head_yaw_joint/cmd@std_msgs/msg/Float64@gz.msgs.Double',
                    f'/model/{robot_name}/joint/camera_tilt_joint/cmd@std_msgs/msg/Float64@gz.msgs.Double'
                ]
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        )
    ] + spawn_robots + bridge_camera_topics + bridge_joint_topics)

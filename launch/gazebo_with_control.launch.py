from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Caminhos relativos ao seu workspace
    workspace_dir = os.path.expanduser("~/ros2_ws/src/guided_navigation")
    urdf_file = os.path.join(workspace_dir, "urdf/camera_robot.urdf")
    ros2_control_yaml = os.path.join(workspace_dir, "urdf/ros2_control.yaml")

    # Comando para rodar o Gazebo diretamente
    gazebo_cmd = [
        'gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'
    ]

    # Node para spawnar o robô
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'camera_robot',
            '-file', urdf_file,
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )
    # Node do robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # Plugin de controle passado como argumento
    ros2_control_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            '--controller-manager', '/controller_manager',
            '--param-file', ros2_control_yaml
        ],
        output='screen'
    )

    # Spawners para os controladores
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    revolute_joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['revolute_joint_controller', '--controller-manager', '/controller_manager']
    )

    return LaunchDescription([
        ExecuteProcess(cmd=gazebo_cmd, output='screen'),
        spawn_entity,
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        revolute_joint_controller_spawner,
    ])

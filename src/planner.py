#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from mapa import Mapa


class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.status_subscriber = self.create_subscription(Bool, '/robot_status', self.status_callback, 10)
        self.particle_filter_subscriber = self.create_subscription(Pose, '/filter_final_pose', self.filter_callback, 10)
        self.command_publisher = self.create_publisher(Int32, '/nav_command', 10)
        self.robot_status = None

        self.get_logger().info('Planner inicializado. Enviando comandos para o Navigation...')        
        self.mapa = Mapa()

    def send_command(self, command):
        command_msg = Int32()
        command_msg.data = command
        self.command_publisher.publish(command_msg)
        self.get_logger().info(f'Comando enviado: {command}')

    def status_callback(self, msg):
        self.robot_status = msg.data
        if msg.data:
            self.get_logger().info('Robô está se movendo.')
        else:
            self.get_logger().info('Robô parou.')

    def filter_callback(self, msg):
        # aqui comeca a separacao das regioes do mapa
        # para escolher qual acao tomar
        if(not self.robot_status):
            self.send_command(1)  # Exemplo: 1 = mover para frente            

def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

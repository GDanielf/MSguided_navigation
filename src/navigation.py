#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        # robot parado: True: em movimento, False: parado
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher = self.create_publisher(Bool, '/robot_status', 10)
        self.command_subscriber = self.create_subscription(Int32, '/nav_command', self.command_callback, 10)
        self.tal = 5.0

        self.moving = False
        

    def command_callback(self, msg):
        # Recebe um comando do Planner e executa a ação correspondente
        command = msg.data
        self.get_logger().info(f'Comando recebido: {command}')
        
        if command == 1:
            self.move_forward()
        elif command == 2:
            self.rotate()
        else:
            self.get_logger().info('Comando desconhecido.')

    def move_forward(self):
        cmd = Twist()
        cmd.linear.x = 1.0  # Mover para frente com velocidade linear
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)
        
        self.moving = True
        self.publish_status()
        self.get_logger().info('Movendo para frente...')
        
        self.create_timer(self.tal, self.stop_robot)  # Para após 3 segundos, simulação

    def rotate(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5  # Rotacionar com velocidade angular
        self.cmd_vel_publisher.publish(cmd)

        self.moving = True
        self.publish_status()
        self.get_logger().info('Rotacionando...')
        
        self.create_timer(self.tal, self.stop_robot)  # Para após 3 segundos, simulação

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd)

        self.moving = False
        self.publish_status()
        self.get_logger().info('Robô parou.')

    def publish_status(self):
        # Publica o status do robô (True: em movimento, False: parado)
        status_msg = Bool()
        status_msg.data = self.moving
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    navigation = Navigation()
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from guided_navigation.msg import PoseEstimate
import math


class ComparePosition(Node):
    def __init__(self):
        super().__init__('compare_position')

        # Subscribers para os dois tópicos de odometria
        self.subscriber_husky = self.create_subscription(
            Odometry,
            '/model/marble_husky_sensor_config_5/odometry',
            self.husky_callback,
            10
        )

        self.subscriber_triangulation = self.create_subscription(
            PoseEstimate,
            'pose_estimate',  # Substitua pelo nome do segundo tópico
            self.triangulation_callback,
            10
        )

        # Variáveis para armazenar as posições
        self.husky_odom_position = None
        self.triangulation_position = [0, 0]

    def husky_callback(self, msg):
        self.husky_odom_position = msg.pose.pose.position
        self.compare_positions()

    def triangulation_callback(self, msg):
        self.triangulation_position[0] = msg.x
        self.triangulation_position[1] = msg.y
        self.compare_positions()

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def compare_positions(self):
        # Certifique-se de que ambas as posições estão disponíveis
        if self.husky_odom_position is not None and self.triangulation_position is not None:
            x_diff = self.husky_odom_position.x - self.triangulation_position[0]
            y_diff = self.husky_odom_position.y - self.triangulation_position[1]
            erro = self.distance(self.husky_odom_position.x, self.husky_odom_position.y, 
                                                           self.triangulation_position[0], self.triangulation_position[1])
            # Aqui você pode fazer o que precisar com as diferenças
            self.get_logger().info(f'Posição da Odom: x={self.husky_odom_position.x}, y={self.husky_odom_position.y}')
            self.get_logger().info(f'Posição Estimada: x={self.triangulation_position[0]}, y={self.triangulation_position[1]}')
            self.get_logger().info(f'Diferenças: dx={x_diff}, dy={y_diff}')
            self.get_logger().info(f'ERRO = {erro}')


def main(args=None):
    rclpy.init(args=args)
    compare_position = ComparePosition()
    rclpy.spin(compare_position)
    compare_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

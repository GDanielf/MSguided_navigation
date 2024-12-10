#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from guided_navigation.msg import PoseEstimate
from geometry_msgs.msg import PoseArray
import math


class ComparePosition(Node):
    def __init__(self):
        super().__init__('compare_position')

        # Variáveis para armazenar posições
        self.last_pose = None
        self.triangulation_position = [0,0]

        # Subscribers para o topico de pose real
        self.subscription = self.create_subscription(
            PoseArray,
            '/model/marble_husky_sensor_config_5/pose',
            self.pose_callback,
            10
        )

        self.subscriber_triangulation = self.create_subscription(
            PoseEstimate,
            'pose_estimate',  # Substitua pelo nome do segundo tópico
            self.triangulation_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.compare_positions)  # A cada 0.1s (10 Hz)
        

    def pose_callback(self, msg):
        if msg.poses:
            self.last_pose = msg.poses[-1]

    def triangulation_callback(self, msg):
        self.triangulation_position[0] = msg.x
        self.triangulation_position[1] = msg.y
        self.compare_positions()

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def compare_positions(self):
        if self.last_pose is not None and self.triangulation_position is not None:
            x_diff = self.last_pose.position.x - self.triangulation_position[0]
            y_diff = self.last_pose.position.y - self.triangulation_position[1]
            erro = self.distance(self.last_pose.position.x, self.last_pose.position.y, 
                                                           self.triangulation_position[0], self.triangulation_position[1])
            
            self.get_logger().info(f'Posição Real: x={self.last_pose.position.x}, y={self.last_pose.position.y}')
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

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from guided_navigation.msg import PoseEstimate
from geometry_msgs.msg import PoseArray
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from std_msgs.msg import Bool, Float64
import time


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
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.path = Path()
        self.pub_real_rqt = self.create_publisher(Point, "/rqt_real_position", 10)
        self.pub_est_rqt = self.create_publisher(Point, "/rqt_estimated_position", 10)    
        self.pub_erro_rqt = self.create_publisher(Float64, "/rqt_erro", 10)      

        # Variáveis para armazenar os valores recebidos
        self.real_pose = None
        self.estimated_pose = None
        self.error_value = None

    def pose_callback(self, msg):
        if msg.poses:
            self.last_pose = msg.poses[-1]
            rqt_point = Point()
            rqt_point.x = self.last_pose.position.x
            rqt_point.y = self.last_pose.position.y
            rqt_point.z = 0.0
            self.pub_real_rqt.publish(rqt_point)
            self.real_pose = (rqt_point.x, rqt_point.y)
            
    def triangulation_callback(self, msg):
        self.triangulation_position[0] = msg.x
        self.triangulation_position[1] = msg.y
        point_msg = Point()
        point_msg.x = float(self.triangulation_position[0])
        point_msg.y = float(self.triangulation_position[1])
        point_msg.z = 0.0
        self.pub_est_rqt.publish(point_msg)
        self.estimated_pose = (point_msg.x, point_msg.y)
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
            erro_msg = Float64()
            erro_msg.data = erro
            self.pub_erro_rqt.publish(erro_msg)


    def destroy_node(self):
        self.file.close()  # Fecha o arquivo ao encerrar o nó
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)    
    node = ComparePosition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

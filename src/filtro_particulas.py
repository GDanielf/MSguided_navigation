#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from guided_navigation.msg import PoseEstimate

class FiltroParticulas(Node):
    def __init__(self):
        super().__init__('filtro_particulas')

        # Subscriber para o tópico /data_topic
        self.subscription = self.create_subscription(
            PoseEstimate,
            '/pose_estimate',
            self.filtro_callback,
            10
        )        

        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        timer_period = 0.5  # 500 ms
        self.timer = self.create_timer(timer_period, self.publish_point)
        self.point_id = 0
        # Definindo o ponto estimado
        self.point = Point()
        self.point.x = 0.0  
        self.point.y = 0.0  
        self.point.z = 0.0

    def filtro_callback(self, msg):
        self.point.x = msg.x
        self.point.y = msg.y      
        self.point.z = 0.0  

    def publish_point(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Certifique-se de que o frame_id esteja correto
        marker.ns = "single_point"
        marker.id = 0  # Mantenha o mesmo ID para substituir o ponto anterior
        marker.type = Marker.SPHERE  # Ou Marker.POINTS, mas um único ponto será suficiente
        marker.action = Marker.ADD 
        # Adicionar o ponto atualizado
        marker.pose.position.x = self.point.x
        marker.pose.position.y = self.point.y
        marker.pose.position.z = self.point.z
        # Definindo a cor e tamanho
        marker.scale.x = 0.2  # Tamanho do ponto
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # Opacidade total
        marker.color.r = 1.0  # Cor vermelha
        marker.color.g = 0.0
        marker.color.b = 0.0
            
        self.publisher_.publish(marker)
        # Apagar o ponto antigo se necessário
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL  # Remove qualquer marcador anterior
        self.publisher_.publish(delete_marker)

def main(args=None):
    rclpy.init(args=args)
    node = FiltroParticulas()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

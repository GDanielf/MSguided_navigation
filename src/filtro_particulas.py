#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from guided_navigation.msg import PoseEstimate
import random
from mapa import Mapa

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
        #inicializacao filtro de particulas
        self.particle_number = 5
        mapa = Mapa()
        self.points_array = mapa.gerar_pontos_aleatorios_dentro(self.particle_number)
        self.publisher_filtro = self.create_publisher(Marker, 'topico_filtro', 10)   
        # Definindo o ponto estimado
        self.publisher_ponto_est = self.create_publisher(Marker, 'visualization_marker', 10)
        timer_period = 0.5  # 500 ms
        self.timer = self.create_timer(timer_period, self.publish_ponto_pose_estimada)
        self.point_id = 0       
        self.point = Point()
        self.point.x = 0.0  
        self.point.y = 0.0  
        self.point.z = 0.0

    def filtro_callback(self, msg):
        self.point.x = msg.x
        self.point.y = msg.y      
        self.point.z = 0.0  

    def publish_ponto_pose_estimada(self):
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
        self.publish_points_array()
        # Definindo a cor e tamanho
        marker.scale.x = 0.2  # Tamanho do ponto
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0  # Opacidade total
        marker.color.r = 1.0  # Cor vermelha
        marker.color.g = 0.0
        marker.color.b = 0.0
            
        self.publisher_ponto_est.publish(marker)
        # Apagar o ponto antigo se necessário
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL  # Remove qualquer marcador anterior
        self.publisher_ponto_est.publish(delete_marker)

    def publish_points_array(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "filtro_points"
        marker.id = 1  # ID diferente do ponto único
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # Definir o array de pontos
        marker.points = self.points_array  # Usar os 500 pontos
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        # Publicar o array de pontos
        self.publisher_filtro.publish(marker)

        # Limpar os pontos antigos, caso necessário
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL  # Remove os pontos antigos
        self.publisher_filtro.publish(delete_marker)

    def update_points_array(self):
        # Atualizar as posições dos 500 pontos
        for point in self.points_array:
            point.x = random.uniform(-10.0, 10.0)  # Novas posições aleatórias
            point.y = random.uniform(-10.0, 10.0)
            point.z = 0.0
    
        # Publicar o array atualizado
        self.publish_points_array()



def main(args=None):
    rclpy.init(args=args)
    node = FiltroParticulas()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

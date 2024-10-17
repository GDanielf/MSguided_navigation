#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from guided_navigation.msg import PoseEstimate
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
import random
import math
from mapa import Mapa

class FiltroParticulas(Node):
    def __init__(self):
        super().__init__('filtro_particulas')
        #subscriber da odom
        self.subscription_navigation = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.navigation_callback,
            10
        )
        self.subscription_navigation  # Impede que o garbage collector elimine a subscrição

        self.robot_status_subscription = self.create_subscription(
            Bool,
            '/robot_status',
            self.robot_status_callback,
            10
        )

        # Subscriber para o tópico /pose_estimate_1 recebe msg a cada 10 seg
        self.subscription_pose_atual = self.create_subscription(
            PoseEstimate,
            '/pose_estimate',
            self.pose_callback,
            10
        )  
        self.publisher_final_pose= self.create_publisher(Point, '/filter_final_pose', 10)  

        #inicializacao filtro de particulas
        self.validar_primeira_msg = True
        mapa = Mapa()        
        self.particle_number = 5
        self.points_array = mapa.gerar_pontos_aleatorios_dentro(self.particle_number)
        self.publisher_filtro = self.create_publisher(Marker, 'visualization_marker', 10)   
        # Definindo o ponto estimado
        self.publisher_ponto_est = self.create_publisher(Marker, 'topic_pose_est', 10)
        #timer_period = 0.5  # 500 ms
        #self.timer = self.create_timer(timer_period, self.publish_ponto_pose_estimada)
        self.ponto_atual = [0.0, 0.0]
        velocidade_linear = 0.0
        velocidade_angular = 0.0
        self.robot_status = None
    
    #acho q isso aqui vai ser uma mensagem recebida pela navegacao para atualizar/prever a posicao
    def robot_status_callback(self, msg):
        self.robot_status = msg.data

    def navigation_callback(self, msg):
        # Obtendo a velocidade linear e angular do robô
        velocidade_linear = msg.linear.x
        velocidade_angular = msg.angular.z


    def pose_callback(self, msg): 
        #A mensagem eh recebida quando o robo estiver parado     
        self.ponto_atual[0] = msg.x
        self.ponto_atual[1] = msg.y 
        self.publish_ponto_pose_estimada()
        self.make_particles()
        self.predict_particles()
        self.update_particles()
        self.resampling_particles()

    #publica o ponto da pose estimada no rviz
    def publish_ponto_pose_estimada(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Certifique-se de que o frame_id esteja correto
        marker.ns = "single_point"
        marker.id = 0  # Mantenha o mesmo ID para substituir o ponto anterior
        marker.type = Marker.SPHERE  # Ou Marker.POINTS, mas um único ponto será suficiente
        marker.action = Marker.ADD 
        # Adicionar o ponto atualizado
        marker.pose.position.x = self.ponto_atual[0]
        marker.pose.position.y = self.ponto_atual[1]
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

    def make_particles(self):
        if(self.validar_primeira_msg):
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
            self.validar_primeira_msg = False

    def predict_particles(self):
        #recebe a velocidade 
        #modelo do carrinho da udacity
        if not self.validar_primeira_msg and self.robot_status:
            pass

    def update_particles(self):
        # Se for o primeiro ponto ou se o ponto atual for diferente do anterior
        if not self.validar_primeira_msg and not self.robot_status:
            # Atualizar as posições
            for point in self.points_array:
                point.x = random.uniform(-10.0, 10.0)  # Novas posições aleatórias
                point.y = random.uniform(-10.0, 10.0)
        else:
            self.get_logger().info('O ponto não mudou, está parado.')        

    def resampling_particles(self, points_array):
        self.publish_point(points_array)

    def publish_point(self, points_array):
        # Criando a mensagem Point
        point_final = Point()
        x = 0
        y = 0
        for point in points_array:
            x = x + point.x
            y = y + point.y
        point_final.x = x/(len(points_array))
        point_final.y = y/(len(points_array))
        # Publicando o ponto no tópico
        self.publisher_.publish(point_final)
        self.get_logger().info(f'Ponto enviado: ({point_final.x}, {point_final.y})')
    
def main(args=None):
    rclpy.init(args=args)
    node = FiltroParticulas()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
from guided_navigation.msg import PoseEstimate
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool
import random
import math
from mapa import Mapa
from particle import Particle
from math import *


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
        self.publisher_final_pose= self.create_publisher(Pose, '/filter_final_pose', 10)  

        #inicializacao filtro de particulas
        self.validar_primeira_msg = True
        mapa = Mapa()        
        self.particle_number = 500
        self.publisher_filtro = self.create_publisher(Marker, 'visualization_marker', 10)
        #fazendo as particulas
        bearing_noise  = 0.05 # initialize bearing noise to zero
        steering_noise = 0.1 # initialize steering noise to zero
        distance_noise = 1.0 
        self.p = []
        for i in range(self.particle_number):
            r = Particle()
            r.set_noise(bearing_noise, steering_noise, distance_noise)
            self.p.append(r)   
        # Definindo o ponto estimado
        self.publisher_ponto_est = self.create_publisher(Marker, 'topic_pose_est', 10)
        #timer_period = 0.5  # 500 ms
        #self.timer = self.create_timer(timer_period, self.publish_ponto_pose_estimada)
        self.ponto_atual = [0.0, 0.0]
        self.velocidade_linear = 0.0
        self.velocidade_angular = 0.0
        self.robot_status = None
        self.get_logger().info('Filtro de particulas inicializado.')        


    def particle_filter(self, particle_number):        
        if(not self.robot_status and self.validar_primeira_msg):
            self.publish_ponto_pose_estimada()
            self.publish_particles(self.p)
            self.validar_primeira_msg = False
        elif(not self.robot_status and not self.validar_primeira_msg):
            # motion update (prediction)
            p2 = []
            for i in range(particle_number):
                p2.append(self.p[i].move(self.velocidade_linear, self.angular_velocity, tal = 5))
            self.p = p2
            # measurement update
            w = []
            for i in range(particle_number):
                w.append(self.p[i].measurement_prob(self.ponto_atual))
            # resampling
            p3 = []
            index = int(random.random() * particle_number)
            beta = 0.0
            mw = max(w)
            for i in range(particle_number):
                beta += random.random() * 2.0 * mw
                while beta > w[index]:
                    beta -= w[index]
                    index = (index + 1) % particle_number
                p3.append(self.p[index])
            self.p = p3
            self.publish_ponto_pose_estimada()
            self.publish_particles(self.p)
            self.publish_final_point(self.p)

    #acho q isso aqui vai ser uma mensagem recebida pela navegacao para atualizar/prever a posicao
    def robot_status_callback(self, msg):
        self.robot_status = msg.data

    def navigation_callback(self, msg):
        # Obtendo a velocidade linear e angular do robô
        self.velocidade_linear = msg.linear.x
        self.velocidade_angular = msg.angular.z

    def pose_callback(self, msg): 
        #A mensagem eh recebida quando o robo estiver parado     
        self.ponto_atual[0] = msg.x
        self.ponto_atual[1] = msg.y 
        self.particle_filter(self.particle_number)

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

    def publish_particles(self, points_array):    
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "filtro_points"
        marker.id = 1  # ID diferente do ponto único
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Definir o array de pontos
        marker.points = points_array  # Usar os 500 pontos
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        for point in enumerate(points_array):
            x, y, theta = point[0], point[1], point[2]
            # Definir a posição
            marker.pose.position.x = x
            marker.pose.position.y = y
            # Converter o ângulo theta para um quaternion (rotação em torno do eixo Z)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(theta / 2.0)  # Parte Z do quaternion
            marker.pose.orientation.w = math.cos(theta / 2.0)  # Parte W do quaternion

            # Publicar o marcador
            self.marker_pub.publish(marker)
            marker.id += 1  # Incrementar o ID para cada marcador

        self.get_logger().info(f'Publicando {len(points_array)} partículas')

        # Publicar o array de pontos
        self.publisher_filtro.publish(marker)

        # Limpar os pontos antigos, caso necessário
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL  # Remove os pontos antigos
        self.publisher_filtro.publish(delete_marker)        
     

    def publish_final_point(self, points_array):
        # Criando a mensagem Point
        point_final = Pose()
        x = 0
        y = 0
        orientation = 0.0
        for i in range(len(points_array)):
            x = x + points_array[i][0]
            y = y + points_array[i][1]
            orientation += (((points_array[i][2] - points_array[0][2] + pi) % (2.0 * pi)) 
                        + points_array[0][2] - pi)
            
        point_final.position.x = x/(len(points_array))
        point_final.position.y = y/(len(points_array))
        point_final.orientation.x = 0.0
        point_final.orientation.y = 0.0
        point_final.orientation.z = math.sin(orientation / 2.0)
        point_final.orientation.w = math.cos(orientation / 2.0)
        # Publicando o ponto no tópico
        self.publisher_final_pose.publish(point_final)
        self.get_logger().info(f'Ponto enviado: ({point_final.position.x}, {point_final.position.y}, {orientation})')
    
def main(args=None):
    rclpy.init(args=args)
    node = FiltroParticulas()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()

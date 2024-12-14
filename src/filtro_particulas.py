#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from guided_navigation.msg import PoseEstimate
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import random
import math
from mapa import Mapa
from particle import Particle
from math import *


class FiltroParticulas(Node):
    def __init__(self):
        super().__init__('filtro_particulas')

        self.subscription_pose_atual = self.create_subscription(
            PoseEstimate,
            '/pose_estimate',
            self.pose_callback,
            10
        )  
        self.subscription_nav_command = self.create_subscription(
            Int32,
            '/robot_commands',
            self.command_callback,
            10
        ) 
        self.comando_recebido = None
        self.publisher_final_pose= self.create_publisher(Pose, '/filter_final_pose', 10)  

        self.movement_status = None

        #inicializacao filtro de particulas
        self.validar_primeira_msg = False
        mapa = Mapa()        
        self.particle_number = 500
        self.publisher_filtro = self.create_publisher(MarkerArray, 'visualization_marker', 10)
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
        self.pose_status = None
        self.get_logger().info('Filtro de particulas inicializado.') 
        self.get_logger().info(f'particulas inicializadas: {self.p[0].orientation}')

    def euler_to_quaternion(self, angulo):
        qx = 0.0
        qy = 0.0
        qz = math.sin(angulo / 2.0)
        qw = math.cos(angulo / 2.0)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)           

    def command_callback(self, msg):
        self.comando_recebido = msg.data

    def pose_callback(self, msg): 
        #A mensagem eh recebida quando o robo estiver parado  
         
        self.ponto_atual[0] = msg.x
        self.ponto_atual[1] = msg.y 
        self.get_logger().info(f'Ponto recebido: {self.ponto_atual}')  
        self.particle_filter(self.particle_number, self.ponto_atual)

    #{0: "parar_robo", 1: "andar_para_frente", 2: "andar_para_tras", 3: "rotacionar_clockwise", 4: "rotacionar_counter_clockwise"}
    def particle_filter(self, particle_number, ponto_recebido): 
        if(not self.validar_primeira_msg):     
            self.publish_ponto_pose_estimada()
            self.publish_particles(self.p)
            self.get_logger().info('If 1.') 
            self.validar_primeira_msg = True
        elif(self.validar_primeira_msg):
            # motion update (prediction)
            self.get_logger().info('If 2.') 
            p2 = []
            for i in range(particle_number):
                if(self.comando_recebido == 0):
                    p2.append(self.p[i].move(0, 0, 2))
                elif(self.comando_recebido == 1):
                    p2.append(self.p[i].move(1.0, 0, 2))
                elif(self.comando_recebido == 2):
                    p2.append(self.p[i].move(-1.0, 0, 2))
                elif(self.comando_recebido == 3):
                    p2.append(self.p[i].move(0.0, -0.5, 2))
                elif(self.comando_recebido == 4):
                    p2.append(self.p[i].move(0.0, 0.5, 2))
            self.p = p2
            # measurement update
            w = []
            for i in range(particle_number):
                w.append(self.p[i].measurement_prob(ponto_recebido))
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
        elif(not self.movement_status and self.validar_primeira_msg):
            self.p = self.p
            self.publish_ponto_pose_estimada()
            self.publish_particles(self.p)
            self.publish_final_point(self.p)     

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
        
        marker_array = MarkerArray()  
        i = 1
        for point in (points_array):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()      
            marker.ns = "filtro_points"
            marker.id = i  # ID diferente do ponto único
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = 0.0  # Mantém-se no plano XY
            marker.pose.orientation = self.euler_to_quaternion(point.orientation)
            # Definir o array de pontos
            marker.scale.x = 1.0
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            i +=1

            marker_array.markers.append(marker)
        
        self.publisher_filtro.publish(marker_array)

        # Limpar os pontos antigos, caso necessário
        #delete_marker = MarkerArray()
        #delete_marker.action = MarkerArray.DELETEALL  # Remove os pontos antigos
        #self.publisher_filtro.publish(delete_marker)        
     

    def publish_final_point(self, points_array):
        # Criando a mensagem Point
        point_final = Pose()
        x = 0
        y = 0
        orientation = 0.0
        for i in range(len(points_array)):
            x = x + points_array[i].x
            y = y + points_array[i].y
            orientation += (((points_array[i].orientation - points_array[0].orientation  + pi) % (2.0 * pi)) 
                        + points_array[0].orientation  - pi)
            
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

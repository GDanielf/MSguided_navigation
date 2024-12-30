#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from guided_navigation.msg import PoseEstimate

from std_msgs.msg import Int32
from particle import Particle
import random
from mapa import Mapa
import math
import matplotlib.pyplot as plt
import numpy as np
import time

class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.subscription_pose_atual = self.create_subscription(
            PoseEstimate,
            '/pose_estimate',
            self.pose_callback,
            10
        ) 
        #self.cmd_vel_subscriber = self.create_subscription(
        #    Twist,
        #    '/cmd_vel',
        #    self.velocity_callback,
        #    10
        #)
        self.robot_mover_publisher = self.create_publisher(Bool, '/robot_moving', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_velocity = 0.0
        self.commands = {
            0: "parar_robo",
            1: "andar_para_frente",
            2: "andar_para_tras",
            3: "rotacionar_clockwise",
            4: "rotacionar_counter_clockwise",
        }
        self.stop_duration = 2.0
        self.rotate_duration = 9
        self.tal = 5.0 
        self.current_timer = None
        self.action_queue = []  # Fila de ações a serem executadas
        #retorno do filtro de particulas
        self.ponto_final = [0.0, 0.0, 0.0]   
        self.mapa = Mapa() 
        self.regiao_objetivo = self.mapa.obter_cor_regiao(5.06, 5.50) 
        self.regiao_antiga = 500
        self.movement_in_progress = True
        self.start_planner = True
        self.contador_de_comando = 0
        self.ponto_atual = [0.0, 0.0]
        self.ultimo_ponto_processado = None 
        self.new_pose_received = False
        self.get_logger().info('Planner inicializado. Enviando comandos para o Navigation...')  
        self.publisher_filtro = self.create_publisher(MarkerArray, 'visualization_marker', 10)
        #definicoes filtro de particula
        self.particle_number = 500
        self.p = []
        for i in range(self.particle_number):
            r = Particle()
            r.set_noise(0.05, 0.087)
            self.p.append(r) 
        self.publish_particles(self.p)   
        self.publisher_ponto_est = self.create_publisher(Marker, 'topic_pose_est', 10)
        self.ponto_antigo = None
        self.new_pose_received = False
        self.start_timer = None
        self.finish_timer = None   
        self.dist = 0      

    #obtem o ponto final para determinar onde o robo deve ir
    def pose_callback(self, msg):    
        self.ponto_atual = [msg.x, msg.y]
        self.dist = msg.dist
        print('ponto recebido: ', self.ponto_atual)
        if(self.start_planner):
            self.start_planner = False
            self.ponto_antigo = self.ponto_atual
            self.publish_particles(self.p)
        self.publish_ponto_pose_estimada()
        #self.get_logger().info(f'Ponto recebido: {self.ponto_atual}')
        #self.particle_filter(self.particle_number, self.ponto_atual)
        #qual regiao esta o ponto?
        regiao_nova_robo = self.mapa.obter_cor_regiao(self.ponto_atual[0], self.ponto_atual[1])        
        #libera particulas
        
        #enquanto o robo nao chegar na regiao objetivo, faca:
        #E a orientacao?
        if(regiao_nova_robo != self.regiao_objetivo):
            #if(regiao_nova_robo >= 32 and regiao_nova_robo <= 79):
            print('if do move') 
            self.move_forward()             
            if self.current_timer:
                self.current_timer.cancel()
            self.current_timer = self.create_timer(self.tal, self.stop)        
            self.contador_de_comando += 1  
            self.p = self.predicao(self.dist, 1)
            self.p = self.reamostragem()
            self.publish_particles(self.p)
            #self.plot_particles(self.p)
            #print(len(self.p))                    
        else:
            print('chegou')
            self.stop()
            #obter direcao 
            #rotacionar o robo 
            self.contador_de_comando += 1 
    
    #funcoes parciais do filtro de particulas
    #{0: "parar_robo", 1: "andar_para_frente", 2: "andar_para_tras", 3: "rotacionar_clockwise", 4: "rotacionar_counter_clockwise"}
    def predicao(self, dist_ponto, acao):
        #predicao
        p2 = []       
        for i in range(self.particle_number):
            p2.append(self.p[i].move(dist_ponto, acao))
        return p2
    
    def reamostragem(self):
        #print(self.ponto_atual)
        # measurement update
        w = []
        for i in range(self.particle_number):
            w.append(self.p[i].measurement_prob(self.ponto_atual))
        
        #print(w)
        p3 = []
        index = int(random.random() * self.particle_number)
        beta = 0.0
        mw = max(w)
        for i in range(self.particle_number):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % self.particle_number
            p3.append(self.p[index])

        return p3
    

    #comandos para enviar para o robo
    def moving_status(self, status):
        msg = Bool()
        msg.data = status
        self.robot_mover_publisher.publish(msg)

    def velocity_sender(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear  
        cmd.angular.z = angular
        self.cmd_vel_publisher.publish(cmd)        

    def schedule_action(self, action, duration):
        """Agenda a execução de uma ação com um timer."""
        if self.current_timer:
            self.current_timer.cancel()
        self.current_timer = self.create_timer(duration, action)        

    def add_action_to_queue(self, action):
        """Adiciona uma ação à fila de ações."""
        self.action_queue.append(action)
        self.process_next_action()

    def process_next_action(self):
        """Processa a próxima ação na fila, se houver uma e não houver outra em andamento."""
        if not self.movement_in_progress and self.action_queue:
            next_action = self.action_queue.pop(0)
            next_action()         

    def stop(self):     
        self.finish_timer = time.time()     
        self.get_logger().info(f'Robo parado')
        self.velocity_sender(0.0, 0.0)
        self.movement_in_progress = False
        self.moving_status(self.movement_in_progress)

    def move_forward(self):        
        self.start_timer = time.time() 
        self.get_logger().info(f'Movendo o robo para frente')
        self.velocity_sender(1.0, 0.0)  
        self.movement_in_progress = True
        self.moving_status(self.movement_in_progress)
        

    def move_backward(self):
        self.get_logger().info(f'Movendo o robo para tras')
        self.velocity_sender(-1.0, 0.0)
        self.schedule_action(self.stop, self.tal) 
        self.movement_in_progress = True

    def rotate_clockwise(self):
        self.get_logger().info(f'Rotacionando no sentido horario')
        self.velocity_sender(0.0, -0.5)
        self.schedule_action(self.stop, self.rotate_duration)
        self.movement_in_progress = True

    def rotate_counter_clockwise(self):
        self.get_logger().info(f'Rotacionando no sentido anti-horario')
        self.velocity_sender(0.0, 0.5)
        self.schedule_action(self.stop, self.rotate_duration)
        self.movement_in_progress = True

    def euler_to_quaternion(self, angulo):
        qx = 0.0
        qy = 0.0
        qz = math.sin(angulo / 2.0)
        qw = math.cos(angulo / 2.0)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)  

    def publish_ponto_pose_estimada(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Certifique-se de que o frame_id esteja correto
        marker.header.stamp = self.get_clock().now().to_msg()
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
        delete_markers = MarkerArray()

        # Remover marcadores antigos
        for old_marker in range(len(self.p)):
            marker = Marker()
            marker.action = Marker.DELETE
            marker.id = old_marker + 1
            delete_markers.markers.append(marker)
        self.publisher_filtro.publish(delete_markers)

        i = 1
        for point in points_array:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()      
            marker.ns = "filtro_points"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = 0.0
            marker.pose.orientation = self.euler_to_quaternion(point.orientacao)
            marker.scale.x = 1.0
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            i += 1

            marker_array.markers.append(marker)

        self.publisher_filtro.publish(marker_array)   

        
    def plot_particles(self, points_array):
        """
        Plota as partículas usando Matplotlib.

        :param points_array: Lista de partículas, onde cada partícula tem atributos x, y e orientacao.
        """
        # Criar uma nova figura
        plt.figure(figsize=(8, 8))
        plt.title("Distribuição de Partículas")
        plt.xlabel("X")
        plt.ylabel("Y")

        # Extrair as posições e orientações
        x_positions = [point.x for point in points_array]
        y_positions = [point.y for point in points_array]

        # Adicionar as partículas ao gráfico como pontos
        plt.scatter(x_positions, y_positions, c='blue', label="Partículas", alpha=0.7)

        # Configurações adicionais
        plt.grid(True)
        plt.axhline(0, color='black', linewidth=0.5)
        plt.axvline(0, color='black', linewidth=0.5)
        plt.legend()
        plt.axis('equal')  # Escala igual para X e Y
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

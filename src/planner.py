#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from guided_navigation.msg import PoseEstimate
from geometry_msgs.msg import PoseArray
from scipy.spatial.transform import Rotation

from std_msgs.msg import Int32
from particle import Particle
import random
from mapa import Mapa
import math
import matplotlib.pyplot as plt
import numpy as np
import time
import copy

class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.subscription_pose_estimate = self.create_subscription(
            PoseEstimate,
            '/pose_estimate',
            self.pose_callback,
            10
        ) 
        self.simulation_subscriber = self.create_subscription(
            Bool,
            '/simulation_status',
            self.simulation_callback,
            10
        )
        self.subscription = self.create_subscription(
            PoseArray,
            '/model/marble_husky_sensor_config_5/pose',
            self.robot_real_pose_callback,
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
        self.rotate_duration = 18
        self.tal = 5.0 
        self.current_timer = None
        self.action_queue = []  # Fila de ações a serem executadas
        #retorno do filtro de particulas
        self.ponto_final = [0.0, 0.0, 0.0]   
        self.mapa = Mapa() 
        self.ponto_objetivo = [5.06, 5.50]
        self.regiao_objetivo = self.mapa.obter_cor_regiao(self.ponto_objetivo[0], self.ponto_objetivo[1]) 
        self.regiao_antiga = 500
        self.movement_in_progress = True
        self.start_planner = True
        self.contador_de_comando = 0
        self.ponto_atual = [0.0, 0.0]
        self.ultimo_ponto_processado = None 
        self.new_pose_received = False
        self.get_logger().info('Planner inicializado. Enviando comandos para o Navigation...')  
        self.publisher_filtro = self.create_publisher(MarkerArray, 'visualization_marker', 10)
        #starvars
        self.m = 16
        #definicoes filtro de particula
        self.particle_number = 1000
        self.p = []
        self.publisher_ponto_est = self.create_publisher(Marker, 'topic_pose_est', 10)
        self.publisher_real_pose = self.create_publisher(Marker, 'topic_real_pose', 10)

        self.ponto_antigo = None
        self.new_pose_received = False
        self.direcao = 0
        #robo real
        self.robot_real_pose = None

    def simulation_callback(self, msg):        
        if(self.start_planner):
            self.start_planner = False              
            for i in range(self.particle_number):
                #(ruido_frente, ruido_virar, sigma_atualizacao, sigma_translacao, tamanho = 30.0)
                self.p.append(Particle(0.5, 0.025, 0.5, 0.5))           
            self.publish_particles(self.p)   

    def robot_real_pose_callback(self, msg):
        if msg.poses:
            self.robot_real_pose = msg.poses[-1]

        self.publish_real_robot_pose()

    #obtem o ponto final para determinar onde o robo deve ir
    def pose_callback(self, msg):    
        self.ponto_atual = [msg.x, msg.y]
        self.ponto_antigo = self.ponto_atual
        #self.publish_ponto_pose_estimada()
        self.publish_particles(self.p)   
        #self.get_logger().info(f'Ponto recebido: {self.ponto_atual}')
        #self.particle_filter(self.particle_number, self.ponto_atual)
        #qual regiao esta o ponto?
        regiao_nova_robo = self.mapa.obter_cor_regiao(self.ponto_atual[0], self.ponto_atual[1])        
        #libera particulas
        
        #enquanto o robo nao chegar na regiao objetivo, faca:
        #E a yaw?
        print('direcao: ', self.direcao)
        if(regiao_nova_robo != self.regiao_objetivo):
            #if(regiao_nova_robo <= 79):
            if(self.direcao == 0 or self.direcao == 4):                               
                #mover o robo
                self.move_forward()             
                if self.current_timer:
                    self.current_timer.cancel()            
                self.current_timer = self.create_timer(self.tal, self.stop)
                self.contador_de_comando += 1  
                #filtro de particula
                self.filtro_de_particulas(0)
                self.publish_ponto_pose_estimada()
                print('melhor particula: ', self.selecionar_particula(self.p))
                print('media: ', self.obter_ponto_filtro_media(self.p))
                print('pose real: ', self.robot_real_pose.position.x, self.robot_real_pose.position.y, self.robot_real_pose.orientation.z)
            elif(self.direcao == 1):
                #rotacione para esquerda e ande para frente
                self.rotate_counter_clockwise()
                if self.current_timer:
                    self.current_timer.cancel()            
                self.current_timer = self.create_timer(self.rotate_duration, self.stop)
                self.filtro_de_particulas(1.57)
                self.publish_ponto_pose_estimada()

            #if(regiao_nova_robo != self.regiao_antiga and self.regiao_antiga != 500):
            #    self.stop()
            #    self.regiao_antiga = regiao_nova_robo
                    

                #if self.regiao_antiga          
                
                
                #self.plot_particles(self.p)
                #print(len(self.p))                    
        else:
            print('chegou')
            self.stop()
            #obter direcao 
            #rotacionar o robo 
            self.contador_de_comando += 1 

    def filtro_de_particulas(self, rotacao):
        #filtro de particula
        # predicao
        for i in range(self.particle_number):
            self.p[i].move(rotacao)        
        
        # atualizacao
        for i in range(self.particle_number):
            self.p[i].measurement_prob(self.ponto_atual)

        #
        p_nova = []
        for i in range(self.particle_number):
            particula = self.selecionar_particula(self.p)
            particula.x = particula.x + random.gauss(0, 0.5)
            particula.y = particula.y + random.gauss(0, 0.5)
            particula.yaw = particula.yaw + random.gauss(0, 0.025)
            p_nova.append(copy.deepcopy(particula))

        self.p = p_nova 
        self.direcao = self.obter_direcao(self.ponto_atual, self.obter_ponto_filtro_media(self.p)[2], self.ponto_objetivo)
    
    #funcoes parciais do filtro de particulas
    def selecionar_particula(self, lista):
        w_soma = sum([particula.w for particula in lista])
        probs = [particula.w / w_soma for particula in lista]
        return lista[np.random.choice(len(lista), p = probs)]
    #{0: "parar_robo", 1: "andar_para_frente", 2: "andar_para_tras", 3: "rotacionar_clockwise", 4: "rotacionar_counter_clockwise"}  
    
    def obter_ponto_filtro_media(self, lista_particulas):
        x = 0
        y = 0
        yaw_sin = 0
        yaw_cos = 0
        for i in range(len(lista_particulas)):
            x = x + lista_particulas[i].x
            y = y + lista_particulas[i].y
            yaw_sin = yaw_sin + math.sin(lista_particulas[i].yaw)
            yaw_cos = yaw_cos + math.cos(lista_particulas[i].yaw)

        return [x/(len(lista_particulas)), y/(len(lista_particulas)), math.atan2(yaw_sin, yaw_cos)]

    def obter_direcao(self, ponto_estimado, direcao_filtro, ponto_objetivo):
        #divisao de star vars, admitindo que a yaw do filtro esteja normalizada
        esquerda = [(self.m * direcao_filtro)/ 8, (3 * self.m * direcao_filtro)/ 8]
        atras = [(3 * self.m * direcao_filtro)/ 8, (5 * self.m * direcao_filtro)/ 8]
        direita = [(5 * self.m * direcao_filtro)/ 8, (7 * self.m * direcao_filtro)/ 8]
        frente = [(7 * self.m * direcao_filtro)/ 8, (self.m * direcao_filtro)/ 8]

        ponto_inicio = np.array([ponto_estimado[0], ponto_estimado[1]])
        ponto_fim = np.array([ponto_objetivo[0], ponto_objetivo[1]])
        vetor_objetivo = (ponto_fim - ponto_inicio)/np.linalg.norm(ponto_fim - ponto_inicio)

        theta = direcao_filtro + np.arccos(np.dot(np.array([np.cos(direcao_filtro), np.sin(direcao_filtro)]), vetor_objetivo))
        comando = 0
        if theta >= esquerda[0] and theta < esquerda[1]:
            comando = 1
        elif theta >= atras[0] and theta < atras[1]:
            comando = 2
        elif theta >= direita[0] and theta < direita[1]:
            comando = 3
        elif theta >= frente[0] and theta < frente[1]:
            comando = 4
        return comando

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
        self.get_logger().info(f'Robo parado')
        self.velocity_sender(0.0, 0.0)
        self.movement_in_progress = False
        self.moving_status(self.movement_in_progress)

    def move_forward(self):        
        self.get_logger().info(f'Movendo o robo para frente')
        self.velocity_sender(1.0, 0.0)  
        self.movement_in_progress = True
        self.moving_status(self.movement_in_progress)
        

    def move_backward(self):
        self.get_logger().info(f'Movendo o robo para tras')
        self.velocity_sender(-1.0, 0.0)
        self.schedule_action(self.stop, self.tal) 
        self.movement_in_progress = True
        self.moving_status(self.movement_in_progress)

    def rotate_clockwise(self):
        self.get_logger().info(f'Rotacionando no sentido horario')
        self.velocity_sender(0.0, -0.5)
        self.schedule_action(self.stop, self.rotate_duration)
        self.movement_in_progress = True
        self.moving_status(self.movement_in_progress)

    def rotate_counter_clockwise(self):
        self.get_logger().info(f'Rotacionando no sentido anti-horario')
        self.velocity_sender(0.0, 0.5)
        self.schedule_action(self.stop, self.rotate_duration)
        self.movement_in_progress = True
        self.moving_status(self.movement_in_progress)

    def publish_ponto_pose_estimada(self):
        quaternion_euler = Rotation.from_euler('xyz', [0, 0, self.obter_ponto_filtro_media(self.p)[2]]).as_quat()
        quat_msg = Quaternion()
        quat_msg.x = quaternion_euler[0]
        quat_msg.y = quaternion_euler[1]
        quat_msg.z = quaternion_euler[2]
        quat_msg.w = quaternion_euler[3]
        marker = Marker()
        marker.header.frame_id = "map"  # Certifique-se de que o frame_id esteja correto
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "single_point"
        marker.id = 0  # Mantenha o mesmo ID para substituir o ponto anterior
        marker.type = Marker.ARROW  # Ou Marker.POINTS, mas um único ponto será suficiente
        marker.action = Marker.ADD 
        # Adicionar o ponto atualizado
        marker.pose.position.x = self.ponto_atual[0]
        marker.pose.position.y = self.ponto_atual[1]
        marker.pose.position.z = 1.0
        marker.pose.orientation = quat_msg
        # Definindo a cor e tamanho
        marker.scale.x = 1.0  # Tamanho do ponto
        marker.scale.y = 0.125
        marker.scale.z = 0.125
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
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.scale.x = 0.1  # Tamanho do ponto
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0  # Opacidade total
            marker.color.r = 0.0  # Cor vermelha
            marker.color.g = 1.0
            marker.color.b = 0.0
            i += 1

            marker_array.markers.append(marker)

        self.publisher_filtro.publish(marker_array)   

    def publish_real_robot_pose(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Certifique-se de que o frame_id esteja correto
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_point"
        marker.id = 0  # Mantenha o mesmo ID para substituir o ponto anterior
        marker.type = Marker.ARROW  # Ou Marker.POINTS, mas um único ponto será suficiente
        marker.action = Marker.ADD 
        # Adicionar o ponto atualizado
        marker.pose.position.x = self.robot_real_pose.position.x
        marker.pose.position.y = self.robot_real_pose.position.y
        marker.pose.position.z = 1.0
        marker.pose.orientation = self.robot_real_pose.orientation
        # Definindo a cor e tamanho
        marker.scale.x = 1.0  # Tamanho do ponto
        marker.scale.y = 0.125
        marker.scale.z = 0.125
        marker.color.a = 1.0  # Opacidade total
        marker.color.r = 1.0  # Cor vermelha
        marker.color.g = 0.0
        marker.color.b = 1.0
            
        self.publisher_real_pose.publish(marker)
        # Apagar o ponto antigo se necessário
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL  # Remove qualquer marcador anterior
        self.publisher_real_pose.publish(delete_marker) 

def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

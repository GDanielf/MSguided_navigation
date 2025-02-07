#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from guided_navigation.msg import PoseEstimate
from geometry_msgs.msg import PoseArray
from particle import Particle
import random
from mapa import Mapa
import math
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
        self.ponto_antigo = None
        self.distance_threshold = 0.1
        self.dist = -10
        self.stop_duration = 10.0
        self.rotate_duration = 10
        self.reverse_duration = 18
        self.tal = 5.0 
        self.espera = 25.0
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
        self.particle_number = 2000
        self.p = []
        self.part_ruido_virar = 0.5
        self.part_sigma_atual = 1.5
        self.part_sigma_translacao = 2.5

        self.variavel_direcao = []
        self.publisher_ponto_est = self.create_publisher(Marker, 'topic_pose_est', 10)
        self.publisher_ponto_objetivo = self.create_publisher(Marker, 'topic_ponto_obj', 10)        
        self.publisher_real_pose = self.create_publisher(Marker, 'topic_real_pose', 10)
        self.publisher_filto_media = self.create_publisher(Marker, 'topic_filtro_media', 10)
        self.direcao_obj_publisher = self.create_publisher(Marker, 'topico_obj_publisher', 10)
        self.publisher_filtro_melhor = self.create_publisher(Marker, 'topic_filtro_melhor', 10)

        self.ponto_antigo = None
        self.new_pose_received = False
        self.direcao_comando = 0
        
        self.melhor_particula= [0.0, 0.0, 0.0]
        self.direcao_array = [0,0,0,0,0,0]
        self.direcao_filtro_melhor = 0
        self.direcao_filtro_media = 0
        #robo real
        self.robot_real_pose = None        

    def distance(self, pose1, pose2):
        return math.sqrt((pose1[0] - pose2[0])**2 + (pose1[1] - pose2[1])**2)

    def simulation_callback(self, msg):        
        if(self.start_planner and msg.data):
            self.publish_ponto_objetivo()
            self.start_planner = False              
            for i in range(self.particle_number):
                #ruido_virar, sigma_atualizacao, sigma_translacao
                self.p.append(Particle(self.part_ruido_virar, self.part_sigma_atual, self.part_sigma_translacao))           
            self.publish_particles(self.p)  
            self.publish_ponto_pose_estimada()  

    def robot_real_pose_callback(self, msg):
        if msg.poses:
            self.robot_real_pose = msg.poses[-1]

        self.publish_real_robot_pose()
        

    #obtem o ponto final para determinar onde o robo deve ir
    def pose_callback(self, msg):    
        self.ponto_atual = [msg.x, msg.y]
        #self.publish_ponto_pose_estimada()
          
        #self.get_logger().info(f'Ponto recebido: {self.ponto_atual}')
        #self.particle_filter(self.particle_number, self.ponto_atual)
        #qual regiao esta o ponto?
        regiao_nova_robo = self.mapa.obter_cor_regiao(self.ponto_atual[0], self.ponto_atual[1])   
        
        #enquanto o robo nao chegar na regiao objetivo, faca:
        #E a yaw?
        
        if(self.ponto_antigo is not None):
            self.dist = self.distance(self.ponto_atual, self.ponto_antigo)
                 
        if(self.ponto_antigo is None or self.dist > self.distance_threshold):
            print("regiao do robo:", regiao_nova_robo, " regiao objetivo: ", self.regiao_objetivo)            
            if(regiao_nova_robo != self.regiao_objetivo):
                #if(regiao_nova_robo <= 79):
                #move pra frente
                print('bagulho doido: ', self.direcao_array)
                print('comando: ', self.direcao_comando)
                if(self.direcao_comando == 0 or self.direcao_comando == 4):                                        
                    #mover o robo                                     
                    self.move_forward()  
                    self.predicao_atualizacao(0)
                    self.publish_rviz()                  
                    self.contador_de_comando += 1 
                #vira para esquerda e vai pra frente 
                elif(self.direcao_comando == 1):
                    #rotacione para esquerda e ande para frente
                    self.rotate_counter_clockwise()
                    self.predicao_atualizacao((math.pi)/2)
                    self.publish_rviz()
                    self.contador_de_comando += 1 
                #Vira 180 para esquerda e vai para frente
                elif(self.direcao_comando == 2):
                    #rotacione para esquerda e ande para frente
                    self.move_backwards()
                    self.predicao_atualizacao(math.pi)
                    self.publish_rviz()
                    self.contador_de_comando += 1
                #vira pra direita e vai pra frente
                elif(self.direcao_comando == 3):
                    #rotacione para esquerda e ande para frente
                    self.rotate_clockwise()
                    self.predicao_atualizacao(-(math.pi)/2)
                    self.publish_rviz()                  
                    self.contador_de_comando += 1 
                self.reamostragem()
                self.publish_rviz()
                #print("Quantidade de comandos: ", self.contador_de_comando)
            else:
                print('chegou')
                self.stop()
                #obter direcao 
                #rotacionar o robo           
            self.ponto_antigo = self.ponto_atual

    def predicao_atualizacao(self, rotacao):
        #filtro de particula
        # predicao
        for i in range(self.particle_number):
            self.p[i].move(rotacao)        
        
        # atualizacao
        for i in range(self.particle_number):
            self.p[i].measurement_prob(self.ponto_atual)

    def reamostragem(self):
        p_nova = []
        for i in range(self.particle_number):
            particula = self.selecionar_particula(self.p)
            particula.x = particula.x + random.gauss(0, 0.5)
            particula.y = particula.y + random.gauss(0, 0.5)
            particula.yaw = particula.yaw + random.gauss(0, 0.5)
            p_nova.append(copy.deepcopy(particula)) 

        self.p = p_nova 
        #selecionar a melhor 
        melhor = self.selecionar_particula(self.p)
        self.melhor_particula = [melhor.x, melhor.y, (melhor.yaw) % (2 * np.pi)]
        self.direcao_filtro_melhor = self.melhor_particula[2]
        #selecionar media
        self.direcao_filtro_media = self.obter_ponto_filtro_media(self.p)[2]
        #usando a melhor
        self.variavel_direcao = self.obter_direcao(self.ponto_atual, self.direcao_filtro_melhor, self.ponto_objetivo)
        self.direcao_comando = self.variavel_direcao[0]
    
    #funcoes parciais do filtro de particulas
    def selecionar_particula(self, lista):
        w_soma = sum([particula.w for particula in lista])
        probs = [particula.w / w_soma for particula in lista]
        return lista[np.random.choice(len(lista), p = probs)]
    #{0: "parar_robo", 1: "andar_para_frente", 2: "andar_para_tras", 3: "rotacionar_clockwise", 4: "rotacionar_counter_clockwise"}  
    
    def obter_ponto_filtro_media(self, lista_particulas):
        x = 0
        y = 0
        x_yaw = 0
        y_yaw = 0
        for i in range(len(lista_particulas)):
            x = x + lista_particulas[i].x
            y = y + lista_particulas[i].y
            x_yaw = x_yaw + np.cos(lista_particulas[i].yaw)
            y_yaw = y_yaw + np.sin(lista_particulas[i].yaw)
        return [x/(len(lista_particulas)), y/(len(lista_particulas)), 
                (np.arctan2((y_yaw/len(lista_particulas)), (x_yaw/len(lista_particulas)))) % (2* math.pi)]

    def obter_direcao(self, ponto_estimado, direcao_filtro, ponto_objetivo):        
        vetor_a = np.array([np.cos(direcao_filtro), np.sin(direcao_filtro)])
        vetor_b = np.array([ponto_objetivo[0], ponto_objetivo[1]]) - np.array([ponto_estimado[0], ponto_estimado[1]])
        produto_escalar = np.dot(vetor_a, vetor_b)
        norma_a = np.linalg.norm(vetor_a)
        norma_b = np.linalg.norm(vetor_b)
        cos_theta = produto_escalar / (norma_a * norma_b)
        theta_objetivo = (np.arccos(cos_theta)) % (2 * math.pi)

        step = ((2 * math.pi) / self.m)
        direcao_virar = (direcao_filtro + theta_objetivo )% (2* math.pi)
        self.publish_direcao_obj(direcao_virar)

        self.direcao_array = [direcao_filtro, theta_objetivo, 
                              (direcao_filtro + step * (self.m / 8)) % (2* math.pi), 
                              (direcao_filtro + step * 3 * (self.m / 8)) % (2* math.pi), 
                              (direcao_filtro + step * 5 * (self.m / 8)) % (2* math.pi), 
                              (direcao_filtro + step * 7 * (self.m / 8)) % (2* math.pi)]

        comando = 0
        #esquerda
        if (self.direcao_array[2] <= direcao_virar < self.direcao_array[3]):
            comando = 1
        #atras
        elif (self.direcao_array[3] <= direcao_virar < self.direcao_array[4]):
            comando = 2
        #direita
        elif (self.direcao_array[4] <= direcao_virar < self.direcao_array[5]):
            comando = 3
        #frente
        elif (self.direcao_array[5] <= direcao_virar < 2* math.pi or direcao_filtro <= theta_objetivo < self.direcao_array[2]):
            comando = 4

        return comando, direcao_filtro, direcao_virar        

    #comandos para enviar para o robo
    def moving_status(self):
        msg = Bool()
        msg.data = self.movement_in_progress
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
        self.moving_status()  

    def move_forward(self):        
        self.get_logger().info(f'Movendo o robo para frente')
        self.velocity_sender(0.5, 0.0) 
        self.schedule_action(self.stop, self.tal) 
        self.movement_in_progress = True  
        self.moving_status() 

    def rotate_clockwise(self):
        self.get_logger().info(f'Rotacionando no sentido horario')
        self.velocity_sender(0.0, -0.5)
        self.schedule_action(self.move_forward, self.rotate_duration) 
        self.movement_in_progress = True
        self.moving_status()             

    def move_backwards(self):
        self.get_logger().info(f'Movendo Para tras')
        self.velocity_sender(0.0, 0.5)
        self.schedule_action(self.move_forward, self.reverse_duration)  
        self.movement_in_progress = True
        self.moving_status()           

    def rotate_counter_clockwise(self):
        self.get_logger().info(f'Rotacionando no sentido anti-horario')
        self.velocity_sender(0.0, 0.5)
        self.schedule_action(self.move_forward, self.rotate_duration)  
        self.movement_in_progress = True
        self.moving_status()    

    def publish_rviz(self):
        self.publish_particles(self.p) 
        self.publish_filtro_media()
        self.publish_ponto_pose_estimada()
        self.publish_filtro_melhor() 

    def publish_ponto_pose_estimada(self):            
        quaternion_euler = self.euler_to_quaternion(0, 0, self.direcao_filtro_melhor)
        
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

    def publish_ponto_objetivo(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Certifique-se de que o frame_id esteja correto
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obj_point"
        marker.id = 0  # Mantenha o mesmo ID para substituir o ponto anterior
        marker.type = Marker.SPHERE  # Ou Marker.POINTS, mas um único ponto será suficiente
        marker.action = Marker.ADD 
        # Adicionar o ponto atualizado
        marker.pose.position.x = self.ponto_objetivo[0]
        marker.pose.position.y = self.ponto_objetivo[1]
        marker.pose.position.z = 1.0
        # Definindo a cor e tamanho
        marker.scale.x = 0.6  # Tamanho do ponto
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        marker.color.a = 1.0  # Opacidade total
        marker.color.r = 0.0  # Cor vermelha
        marker.color.g = 0.0
        marker.color.b = 1.0
            
        self.publisher_ponto_objetivo.publish(marker)
        # Apagar o ponto antigo se necessário
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL  # Remove qualquer marcador anterior
        self.publisher_ponto_objetivo.publish(delete_marker)

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

    def publish_filtro_media(self):
        quaternion_euler_4 = self.euler_to_quaternion(0, 0, self.direcao_filtro_media)
        
        marker = Marker()
        quat_msg_2 = Quaternion()
        quat_msg_2.x = quaternion_euler_4[0]
        quat_msg_2.y = quaternion_euler_4[1]
        quat_msg_2.z = quaternion_euler_4[2]
        quat_msg_2.w = quaternion_euler_4[3]
        marker.header.frame_id = "map"  # Certifique-se de que o frame_id esteja correto
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "filtro_media"
        marker.id = 0  # Mantenha o mesmo ID para substituir o ponto anterior
        marker.type = Marker.ARROW  # Ou Marker.POINTS, mas um único ponto será suficiente
        marker.action = Marker.ADD 
        # Adicionar o ponto atualizado
        marker.pose.position.x = self.obter_ponto_filtro_media(self.p)[0]
        marker.pose.position.y = self.obter_ponto_filtro_media(self.p)[1]
        marker.pose.position.z = 1.0
        marker.pose.orientation = quat_msg_2
        # Definindo a cor e tamanho
        marker.scale.x = 1.0  # Tamanho do ponto
        marker.scale.y = 0.125
        marker.scale.z = 0.125
        marker.color.a = 1.0  # Opacidade total
        marker.color.r = 0.0  # Cor vermelha
        marker.color.g = 1.0
        marker.color.b = 1.0
            
        self.publisher_filto_media.publish(marker)
        # Apagar o ponto antigo se necessário
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL  # Remove qualquer marcador anterior
        self.publisher_filto_media.publish(delete_marker) 

    def publish_direcao_obj(self, direcao_obj):
        quaternion_euler_2 = self.euler_to_quaternion(0, 0, direcao_obj)
        
        marker = Marker()
        quat_msg_2 = Quaternion()
        quat_msg_2.x = quaternion_euler_2[0]
        quat_msg_2.y = quaternion_euler_2[1]
        quat_msg_2.z = quaternion_euler_2[2]
        quat_msg_2.w = quaternion_euler_2[3]
        marker.header.frame_id = "map"  # Certifique-se de que o frame_id esteja correto
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ponto_obj"
        marker.id = 0  # Mantenha o mesmo ID para substituir o ponto anterior
        marker.type = Marker.ARROW  # Ou Marker.POINTS, mas um único ponto será suficiente
        marker.action = Marker.ADD 
        # Adicionar o ponto atualizado
        marker.pose.position.x = self.ponto_atual[0]
        marker.pose.position.y = self.ponto_atual[1]
        marker.pose.position.z = 1.0
        marker.pose.orientation = quat_msg_2
        # Definindo a cor e tamanho
        marker.scale.x = 1.0  # Tamanho do ponto
        marker.scale.y = 0.125
        marker.scale.z = 0.125
        marker.color.a = 1.0  # Opacidade total
        marker.color.r = 0.0  # Cor vermelha
        marker.color.g = 1.0
        marker.color.b = 1.0
            
        self.direcao_obj_publisher.publish(marker)
        # Apagar o ponto antigo se necessário
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL  # Remove qualquer marcador anterior
        self.direcao_obj_publisher.publish(delete_marker) 

    def publish_filtro_melhor(self):
        quaternion_euler_3 = self.euler_to_quaternion(0, 0, self.melhor_particula[2])
        
        marker = Marker()
        quat_msg_2 = Quaternion()
        quat_msg_2.x = quaternion_euler_3[0]
        quat_msg_2.y = quaternion_euler_3[1]
        quat_msg_2.z = quaternion_euler_3[2]
        quat_msg_2.w = quaternion_euler_3[3]
        marker.header.frame_id = "map"  # Certifique-se de que o frame_id esteja correto
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "filtro_melhor"
        marker.id = 0  # Mantenha o mesmo ID para substituir o ponto anterior
        marker.type = Marker.ARROW  # Ou Marker.POINTS, mas um único ponto será suficiente
        marker.action = Marker.ADD 
        # Adicionar o ponto atualizado
        marker.pose.position.x = self.melhor_particula[0]
        marker.pose.position.y = self.melhor_particula[1]
        marker.pose.position.z = 1.0
        marker.pose.orientation = quat_msg_2
        # Definindo a cor e tamanho
        marker.scale.x = 1.0  # Tamanho do ponto
        marker.scale.y = 0.125
        marker.scale.z = 0.125
        marker.color.a = 1.0  # Opacidade total
        marker.color.r = 0.5  # Cor vermelha
        marker.color.g = 1.0
        marker.color.b = 1.0
            
        self.publisher_filtro_melhor.publish(marker)
        # Apagar o ponto antigo se necessário
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL  # Remove qualquer marcador anterior
        self.publisher_filtro_melhor.publish(delete_marker) 

        
    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]
    
def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from guided_navigation.msg import PoseEstimate
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from particle import Particle
import random
from mapa import Mapa
import math
import numpy as np
import copy
from scipy.spatial.transform import Rotation

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
        self.subscription_pose_real = self.create_subscription(
            PoseArray,
            '/model/marble_husky_sensor_config_5/pose',
            self.robot_real_pose_callback,
            10
        )
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/model/marble_husky_sensor_config_5/odometry',
            self.odom_callback,
            10)
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
        self.distance_threshold = 0.2
        self.dist = -10
        self.tal = 3.0 
        self.current_timer = None
        self.action_queue = []  # Fila de ações a serem executadas
        #retorno do filtro de particulas
        self.ponto_final = [0.0, 0.0, 0.0]   
        self.mapa = Mapa() 
        #teste com ponto q deu ruim (robo virando no comeco)
        self.ponto_objetivo = [round(random.uniform(-10, 10), 2), round(random.uniform(-7.5, 7.5), 2)]
        #self.ponto_objetivo = [-9.23, 4.55]
        self.regiao_objetivo = self.mapa.obter_cor_regiao(self.ponto_objetivo[0], self.ponto_objetivo[1]) 
        self.pontos_regiao_objetivo = self.mapa.get_regiao_por_numero(self.regiao_objetivo)
        self.regiao_antiga = 500
        self.movement_in_progress = True
        self.start_planner = True
        self.contador_de_comando = 0
        self.ponto_atual = [0.0, 0.0]
        self.ultimo_ponto_processado = None 
        self.new_pose_received = False
        self.get_logger().info(f'Planner inicializado. Ponto objetivo: {self.ponto_objetivo}')  
        self.publisher_filtro = self.create_publisher(MarkerArray, 'visualization_marker', 10)
        self.publish_regioes_angles = self.create_publisher(MarkerArray, 'regioes_angles_topic', 10)
        #starvars
        self.m = 16
        #definicoes filtro de particula
        self.particle_number = 1000
        self.p = []        
        self.part_ruido_virar = 0.05
        self.part_sigma_atual = 0.5
        self.part_sigma_translacao = 0.5
        for i in range(self.particle_number):
            #ruido_virar, sigma_atualizacao, sigma_translacao
            self.p.append(Particle(self.part_ruido_virar, self.part_sigma_atual, self.part_sigma_translacao)) 

        self.publisher_ponto_est = self.create_publisher(Marker, 'topic_pose_est', 10)
        self.publisher_ponto_objetivo = self.create_publisher(Marker, 'topic_ponto_obj', 10)        
        self.publisher_real_pose = self.create_publisher(Marker, 'topic_real_pose', 10)
        self.publisher_filto_media = self.create_publisher(Marker, 'topic_filtro_media', 10)
        self.direcao_obj_publisher = self.create_publisher(Marker, 'topico_obj_publisher', 10)
        self.regiao_objetivo_publisher = self.create_publisher(Marker, 'topico_regiao_objetivo', 10)
    
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.path = Path()
        self.path.header.frame_id = "map"

        self.ponto_antigo = None
        self.new_pose_received = False
        self.comando_direcao = 0
        self.inicio = True
        
        self.direcao_filtro_media = 0
        #robo real
        self.robot_real_pose = None 
        #controle pela odom
        self.yaw_odom = 0.0
        self.tolerance = math.radians(2)
        self.ajuste_fino = (math.pi/2) + 0.04
        self.kp = 1
        self.permission_to_rotate = False
        self.sentido = 0
        self.target_rotation = 0.0

    def distance(self, pose1, pose2):
        return math.sqrt((pose1[0] - pose2[0])**2 + (pose1[1] - pose2[1])**2)    

    def robot_real_pose_callback(self, msg):
        if msg.poses:
            self.robot_real_pose = msg.poses[-1]

        self.publish_real_robot_pose()

    def simulation_callback(self, msg):
        pass

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]   

    def odom_callback(self, msg):
        quat = msg.pose.pose.orientation
        self.yaw_odom = ((Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])).as_euler('xyz', degrees = False))[2]
        if(self.permission_to_rotate):
            if self.sentido == 1:                
                vel_angular = self.kp
            elif self.sentido == -1:                
                vel_angular = self.kp * -1
            else:
                vel_angular = self.kp * -1
            
            error = abs(self.target_rotation % (2 * math.pi) - self.yaw_odom % (2 * math.pi))
            #print(error, self.target_rotation % (2 * math.pi), self.yaw_odom % (2 * math.pi))
            if abs(error) < self.tolerance:
                self.move_forward()
                self.permission_to_rotate = False
            else:           
                self.velocity_sender(0.0, error * 0.5 * vel_angular)       

    #obtem o ponto final para determinar onde o robo deve ir
    def pose_callback(self, msg):    
        self.ponto_atual = [msg.x, msg.y]
        regiao_nova_robo = self.mapa.obter_cor_regiao(self.ponto_atual[0], self.ponto_atual[1])   
        self.publish_rviz()
        if(self.ponto_antigo is not None):
            self.dist = self.distance(self.ponto_atual, self.ponto_antigo)
        #a direcao a ser tomada pelo robo deve ser calculada assim que receber o ponto estimado
        if(self.ponto_antigo is None or self.dist > self.distance_threshold):
            if(not(self.inicio)):
                self.reamostragem()                
                self.publish_filtro_media()
                self.publish_rviz()
                dicionario_comandos = self.obter_comando_direcao(self.ponto_objetivo, self.p)
                probabilidade_comando = self.comando_probabilidade(dicionario_comandos)
                media_comando = self.obter_comando_direcao_media(self.direcao_filtro_media, self.ponto_atual, self.ponto_objetivo)
                self.comando_direcao = probabilidade_comando
                print(dicionario_comandos, probabilidade_comando, media_comando)
            print("regiao do robo:", regiao_nova_robo, " regiao objetivo: ", self.regiao_objetivo)            
            if(regiao_nova_robo != self.regiao_objetivo):
                print('comando: ', self.comando_direcao)                
                if(self.comando_direcao == 1):
                    self.get_logger().info(f'Rotacionando no sentido anti-horario')
                    self.permission_to_rotate = True
                    self.sentido = 1
                    yaw_antigo = self.yaw_odom % (2 * math.pi)
                    self.target_rotation = yaw_antigo + self.ajuste_fino * self.sentido
                    self.predicao((math.pi)/2)
                    #self.publish_rviz()
                    self.contador_de_comando += 1 
                #Vira 180 para esquerda e vai para frente
                elif(self.comando_direcao == 2):
                    self.get_logger().info(f'Rotacionando 180')
                    self.permission_to_rotate = True
                    self.sentido = 2
                    yaw_antigo = self.yaw_odom % (2 * math.pi)
                    self.target_rotation = yaw_antigo + self.ajuste_fino * self.sentido
                    self.predicao(math.pi)
                    #self.publish_rviz()
                    self.contador_de_comando += 1
                #vira pra direita e vai pra frente
                elif(self.comando_direcao == 3):
                    self.get_logger().info(f'Rotacionando no sentido horario')
                    self.permission_to_rotate = True
                    self.sentido = -1
                    yaw_antigo = self.yaw_odom % (2 * math.pi)
                    self.target_rotation = yaw_antigo + self.ajuste_fino * self.sentido
                    self.predicao(-(math.pi)/2)
                    #self.publish_rviz()                  
                    self.contador_de_comando += 1 
                else:      
                    self.move_forward()  
                    self.predicao(0)
                    #self.publish_rviz()                  
                    self.contador_de_comando += 1 
                    self.inicio = False
                #vira para esquerda e vai pra frente 
            else:
                print('chegou')
                self.publish_rviz()
                self.stop()       
            self.ponto_antigo = self.ponto_atual

    def predicao(self, rotacao):
        # predicao
        for i in range(self.particle_number):
            self.p[i].move(rotacao, 1.5) 

    def reamostragem(self):
        # atualizacao
        for i in range(self.particle_number):
            self.p[i].measurement_prob(self.ponto_atual)

        p_nova = []
        for i in range(self.particle_number):
            particula = self.selecionar_particula(self.p)
            particula.x = particula.x + random.gauss(0, 0.5)
            particula.y = particula.y + random.gauss(0, 0.5)
            particula.yaw = (particula.yaw + random.gauss(0, 0.25)) % (2 * math.pi)
            p_nova.append(copy.deepcopy(particula)) 

        self.p = p_nova 
        self.direcao_filtro_media = self.obter_ponto_filtro_media(self.p)[2]
        #selecionar media        
    
    #funcoes parciais do filtro de particulas
    def selecionar_particula(self, lista):
        w_soma = sum([particula.w for particula in lista])
        probs = [particula.w / w_soma for particula in lista]
        return lista[np.random.choice(len(lista), p = probs)]
    
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

    def regioes_espaciais(self, direcao):
        step = ((2 * math.pi) / self.m)  
        direcao_array = [(direcao + step * (self.m / 8)) % (2* math.pi), 
                              (direcao + step * 3 * (self.m / 8)) % (2* math.pi), 
                              (direcao + step * 5 * (self.m / 8)) % (2* math.pi), 
                              (direcao + step * 7 * (self.m / 8)) % (2* math.pi)]
        return direcao_array

    def obter_comando_direcao(self, lista_ponto_objetivo, lista_filtro):  
        comando = {"1": 0, "2" : 0, "3": 0, "4": 0}
        for particula in lista_filtro:
            vetor_part = np.array([particula.x, particula.y, particula.yaw])
            vetor_obj = np.array([lista_ponto_objetivo[0], lista_ponto_objetivo[1]])
            delta_x = vetor_obj[0] - vetor_part[0]
            delta_y = vetor_obj[1] - vetor_part[1]
            theta_obj = (np.arctan2(delta_y, delta_x)) % (2* np.pi)   
            regioes_espaciais = self.regioes_espaciais(particula.yaw)
            maior_valor = max(regioes_espaciais)
            indice_maior = regioes_espaciais.index(maior_valor)
            if indice_maior == 0:
                if(regioes_espaciais[0] <= theta_obj < 2*math.pi) or (0 <= theta_obj < regioes_espaciais[1]):
                    comando["1"] += 1
                elif((regioes_espaciais[1] <= theta_obj < regioes_espaciais[2])):
                    comando["2"] += 1
                elif((regioes_espaciais[2] <= theta_obj < regioes_espaciais[3])):
                    comando["3"] += 1
                elif((regioes_espaciais[3] <= theta_obj < regioes_espaciais[0])):
                    comando["4"] += 1
            elif indice_maior == 1:
                if((regioes_espaciais[0] <= theta_obj < regioes_espaciais[1])):
                    comando["1"] += 1
                elif(regioes_espaciais[1] <= theta_obj < 2*math.pi) or (0 <= theta_obj < regioes_espaciais[2]):
                    comando["2"] += 1
                elif((regioes_espaciais[2] <= theta_obj < regioes_espaciais[3])):
                    comando["3"] += 1
                elif((regioes_espaciais[3] <= theta_obj < regioes_espaciais[0])):
                    comando["4"] += 1
            elif indice_maior == 2:
                if((regioes_espaciais[0] <= theta_obj < regioes_espaciais[1])):
                    comando["1"] += 1
                elif((regioes_espaciais[1] <= theta_obj < regioes_espaciais[2])):
                    comando["2"] += 1
                elif(regioes_espaciais[2] <= theta_obj < 2*math.pi) or (0 <= theta_obj < regioes_espaciais[3]):
                    comando["3"] += 1
                elif((regioes_espaciais[3] <= theta_obj < regioes_espaciais[0])):
                    comando["4"] += 1
            elif indice_maior == 3:
                if((regioes_espaciais[0] <= theta_obj < regioes_espaciais[1])):
                    comando["1"] += 1
                elif((regioes_espaciais[1] <= theta_obj < regioes_espaciais[2])):
                    comando["2"] += 1
                elif((regioes_espaciais[2] <= theta_obj < regioes_espaciais[3])):
                    comando["3"] += 1
                elif(regioes_espaciais[3] <= theta_obj < 2*math.pi) or (0 <= theta_obj < regioes_espaciais[0]):
                    comando["4"] += 1
            
        return comando
    
    def obter_comando_direcao_media(self, direcao_filtro, ponto_estimado, lista_ponto_objetivo):
        comando = 0
        vetor_part = np.array([ponto_estimado[0], ponto_estimado[1], direcao_filtro])
        vetor_obj = np.array([lista_ponto_objetivo[0], lista_ponto_objetivo[1]])
        delta_x = vetor_obj[0] - vetor_part[0]
        delta_y = vetor_obj[1] - vetor_part[1]
        theta_obj = (np.arctan2(delta_y, delta_x)) % (2* np.pi)   
        regioes_espaciais = self.regioes_espaciais(direcao_filtro)
        print(theta_obj, regioes_espaciais)
        self.publish_regioes_espaciais(regioes_espaciais, ponto_estimado)
        maior_valor = max(regioes_espaciais)
        indice_maior = regioes_espaciais.index(maior_valor)
        if indice_maior == 0:
            if(regioes_espaciais[0] <= theta_obj < 2*math.pi) or (0 <= theta_obj < regioes_espaciais[1]):
                comando = 1
            elif((regioes_espaciais[1] <= theta_obj < regioes_espaciais[2])):
                comando = 2
            elif((regioes_espaciais[2] <= theta_obj < regioes_espaciais[3])):
                comando = 3
            elif((regioes_espaciais[3] <= theta_obj < regioes_espaciais[0])):
                comando = 4
        elif indice_maior == 1:
            if((regioes_espaciais[0] <= theta_obj < regioes_espaciais[1])):
                comando = 1
            elif(regioes_espaciais[1] <= theta_obj < 2*math.pi) or (0 <= theta_obj < regioes_espaciais[2]):
                comando = 2
            elif((regioes_espaciais[2] <= theta_obj < regioes_espaciais[3])):
                comando = 3
            elif((regioes_espaciais[3] <= theta_obj < regioes_espaciais[0])):
                comando = 4
        elif indice_maior == 2:
            if((regioes_espaciais[0] <= theta_obj < regioes_espaciais[1])):
                comando = 1
            elif((regioes_espaciais[1] <= theta_obj < regioes_espaciais[2])):
                comando = 2
            elif(regioes_espaciais[2] <= theta_obj < 2*math.pi) or (0 <= theta_obj < regioes_espaciais[3]):
                comando = 3
            elif((regioes_espaciais[3] <= theta_obj < regioes_espaciais[0])):
                comando = 4
        elif indice_maior == 3:
            if((regioes_espaciais[0] <= theta_obj < regioes_espaciais[1])):
                comando = 1
            elif((regioes_espaciais[1] <= theta_obj < regioes_espaciais[2])):
                comando = 2
            elif((regioes_espaciais[2] <= theta_obj < regioes_espaciais[3])):
                comando = 3
            elif(regioes_espaciais[3] <= theta_obj < 2*math.pi) or (0 <= theta_obj < regioes_espaciais[0]):
                comando = 4
        return comando
    
    def comando_probabilidade(self, dicionario_comando):
        total = sum(dicionario_comando.values())
        prob = {key: value / total for key, value in dicionario_comando.items()}
        max_key = max(prob, key=prob.get)
        return int(max_key)

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

    def publish_rviz(self):
        self.publish_particles(self.p)         
        self.publish_ponto_pose_estimada()
        self.publish_ponto_objetivo()
        self.publish_regiao_objetivo()

    def publish_ponto_pose_estimada(self):       
        marker = Marker()
        marker.header.frame_id = "map"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "single_point"
        marker.id = 0  
        marker.type = Marker.SPHERE 
        marker.action = Marker.ADD         
        marker.pose.position.x = self.ponto_atual[0]
        marker.pose.position.y = self.ponto_atual[1]
        marker.pose.position.z = 1.0 
        marker.scale.x = 0.3 
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0  
        marker.color.r = 1.0  
        marker.color.g = 0.0
        marker.color.b = 0.0            
        self.publisher_ponto_est.publish(marker)        
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL  
        self.publisher_ponto_est.publish(delete_marker) 

    def publish_ponto_objetivo(self):
        marker = Marker()
        marker.header.frame_id = "map"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obj_point"
        marker.id = 0  
        marker.type = Marker.SPHERE 
        marker.action = Marker.ADD         
        marker.pose.position.x = self.ponto_objetivo[0]
        marker.pose.position.y = self.ponto_objetivo[1]
        marker.pose.position.z = 1.0        
        marker.scale.x = 0.6  
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        marker.color.a = 1.0  
        marker.color.r = 0.0  
        marker.color.g = 0.0
        marker.color.b = 1.0            
        self.publisher_ponto_objetivo.publish(marker)        
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL  
        self.publisher_ponto_objetivo.publish(delete_marker)

    def publish_regiao_objetivo(self):
        xmin, xmax, ymin, ymax = self.pontos_regiao_objetivo
        # Criando a mensagem do marcador
        marker = Marker()
        marker.header.frame_id = "map"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "quadrados"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  
        marker.color.a = 1.0  
        marker.color.r = 0.0  
        marker.color.g = 0.0
        marker.color.b = 1.0  
        
        pontos = [
            (xmin, ymin),
            (xmax, ymin),
            (xmax, ymax),
            (xmin, ymax),
            (xmin, ymin) 
        ]
        for x, y in pontos:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            marker.points.append(p)
        self.regiao_objetivo_publisher.publish(marker)

    def publish_particles(self, points_array):             
        marker_array = MarkerArray()
        delete_markers = MarkerArray()        
        for old_marker in range(len(self.p)):
            marker = Marker()
            marker.action = Marker.DELETE
            marker.id = old_marker + 1
            delete_markers.markers.append(marker)
        self.publisher_filtro.publish(delete_markers)
        i = 1
        for point in points_array:
            quaternion = self.euler_to_quaternion(0, 0, point.yaw) 
            marker = Marker()
            quat = Quaternion()
            quat.x = quaternion[0]
            quat.y = quaternion[1]
            quat.z = quaternion[2]
            quat.w = quaternion[3]            
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()      
            marker.ns = "filtro_points"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.orientation = quat
            marker.scale.x = 0.3  
            marker.scale.y = 0.025
            marker.scale.z = 0.025
            marker.color.r = 0.0  
            marker.color.g = 1.0
            marker.color.b = 1.0  
            marker.color.a = 1.0            
            i += 1
            marker_array.markers.append(marker)
        self.publisher_filtro.publish(marker_array) 

    def publish_regioes_espaciais(self, regioes, ponto_desejado):             
        marker_array = MarkerArray()
        delete_markers = MarkerArray()        
        for old_marker in range(len(regioes)):
            marker = Marker()
            marker.action = Marker.DELETE
            marker.id = old_marker + 1
            delete_markers.markers.append(marker)
        self.publish_regioes_angles.publish(delete_markers)
        i = 1
        for angles in regioes:
            quaternion = self.euler_to_quaternion(0, 0, angles) 
            marker = Marker()
            quat = Quaternion()
            quat.x = quaternion[0]
            quat.y = quaternion[1]
            quat.z = quaternion[2]
            quat.w = quaternion[3]            
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()      
            marker.ns = "filtro_points"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = ponto_desejado[0]
            marker.pose.position.y = ponto_desejado[1]
            marker.pose.orientation = quat
            marker.scale.x = 1.0  
            marker.scale.y = 0.125
            marker.scale.z = 0.125
            marker.color.r = 0.25 
            marker.color.g = 0.25
            marker.color.b = 0.5 
            marker.color.a = 1.0            
            i += 1
            marker_array.markers.append(marker)
        self.publish_regioes_angles.publish(marker_array) 

    def publish_filtro_media(self):
        quaternion_euler_4 = self.euler_to_quaternion(0, 0, self.direcao_filtro_media)        
        marker = Marker()
        quat_msg_2 = Quaternion()
        quat_msg_2.x = quaternion_euler_4[0]
        quat_msg_2.y = quaternion_euler_4[1]
        quat_msg_2.z = quaternion_euler_4[2]
        quat_msg_2.w = quaternion_euler_4[3]
        marker.header.frame_id = "map"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "filtro_media"
        marker.id = 0  
        marker.type = Marker.ARROW 
        marker.action = Marker.ADD         
        marker.pose.position.x = self.obter_ponto_filtro_media(self.p)[0]
        marker.pose.position.y = self.obter_ponto_filtro_media(self.p)[1]
        marker.pose.position.z = 0.5
        marker.pose.orientation = quat_msg_2        
        marker.scale.x = 1.0  
        marker.scale.y = 0.125
        marker.scale.z = 0.125
        marker.color.a = 1.0  
        marker.color.r = 0.0  
        marker.color.g = 0.7
        marker.color.b = 0.0    
        self.publisher_filto_media.publish(marker)        
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL  
        self.publisher_filto_media.publish(delete_marker) 

    def publish_real_robot_pose(self):
        # Criando um Marker para visualizar o robô no RViz2
        marker = Marker()
        marker.header.frame_id = "map"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_point"
        marker.id = 0  
        marker.type = Marker.ARROW 
        marker.action = Marker.ADD         
        marker.pose.position.x = self.robot_real_pose.position.x
        marker.pose.position.y = self.robot_real_pose.position.y
        marker.pose.position.z = 1.0
        marker.pose.orientation = self.robot_real_pose.orientation        
        marker.scale.x = 1.0  
        marker.scale.y = 0.125
        marker.scale.z = 0.125
        marker.color.a = 1.0  
        marker.color.r = 1.0  
        marker.color.g = 0.0
        marker.color.b = 1.0      
        self.publisher_real_pose.publish(marker)  
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL  
        self.publisher_real_pose.publish(delete_marker) 
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = marker.header.stamp
        pose_stamped.header.frame_id = marker.header.frame_id
        pose_stamped.pose = self.robot_real_pose
        self.path.poses.append(pose_stamped)
        path_msg = Path()
        path_msg.header.stamp = marker.header.stamp
        path_msg.header.frame_id = marker.header.frame_id
        path_msg.poses = self.path.poses
        self.path_pub.publish(path_msg)        
    
    
def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

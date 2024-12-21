#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int32
from particle import Particle
from mapa import Mapa
import math

class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.subscription_pose_atual = self.create_subscription(
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
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
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
        self.simulation_status = False
        self.movement_in_progress = True
        self.start_planner = True
        self.contador_de_comando = 0
        self.get_logger().info('Planner inicializado. Enviando comandos para o Navigation...')  
        self.publisher_ponto_est = self.create_publisher(Marker, 'topic_pose_est', 10)
        self.publisher_filtro = self.create_publisher(MarkerArray, 'visualization_marker', 10)
        #definicoes filtro de particula
        self.particle_number = 500
        self.p = []
        for i in range(self.particle_number):
            r = Particle()
            r.set_noise(ruido_frente, ruido_virar)
            self.p.append(r)     
        

    def simulation_callback(self, msg):        
        self.simulation_status = msg.data     
        self.start_movement_sequence()

    #obtem o ponto final para determinar onde o robo deve ir
    def pose_callback(self, msg):    
        self.ponto_atual[0] = msg.x
        self.ponto_atual[1] = msg.y
        self.publisher_ponto_est = self.create_publisher(Marker, 'topic_pose_est', 10)
        self.get_logger().info(f'Ponto recebido: {self.ponto_atual}')
        #self.particle_filter(self.particle_number, self.ponto_atual)
        #qual regiao esta o ponto?
        regiao_nova_robo = self.mapa.obter_cor_regiao(self.ponto_atual[0], self.ponto_atual[1])        
        #libera particulas
        self.p = self.p
        self.publish_particles(self.p) 
        sair_loop = True       
        #enquanto o robo nao chegar na regiao objetivo, faca:
        #E a orientacao?
        while(regiao_nova_robo != self.regiao_objetivo and sair_loop):
            #regioa verde e amarela
            while(self.regiao_antiga == regiao_nova_robo)
                if(regiao_nova_robo >= 30 and regiao_nova_robo <= 59):
                    self.add_action_to_queue(self.move_forward) 
                    self.contador_de_comando += 1  
                    self.p = self.predicao_particulas(1, self.p, self.particle_number)   
                    self.p = self.resampling(self.p, self.particle_number, self.ponto_atual) 
                    self.publish_particles(self.p) 
                    sair_loop = False
                    break
            if(self.start_planner):
                sair_loop = False
                self.regiao_antiga = regiao_nova_robo
                break
            else:
                self.add_action_to_queue(self.stop)  
                #obter direcao 
                #rotacionar o robo 
                self.p = self.predicao_particulas(1, self.p, self.particle_number)   
                self.p = self.resampling(self.p, self.particle_number, self.ponto_atual) 
                self.contador_de_comando += 1 
                self.regiao_antiga = regiao_nova_robo
                sair_loop = False
                break
    
    #funcoes parciais do filtro de particulas
    #{0: "parar_robo", 1: "andar_para_frente", 2: "andar_para_tras", 3: "rotacionar_clockwise", 4: "rotacionar_counter_clockwise"}
    def predicao_particulas(self, acao, lista_particula, particle_number):
        p2 = []         
        if(self.comando_recebido == 0):
            p2 = lista_particula
        elif(self.comando_recebido == 1):
            for i in range(particle_number):
                p2.append(lista_particula.move(1.0, 0, 2))
        elif(self.comando_recebido == 2):
            for i in range(particle_number):
                p2.append(lista_particula.move(-1.0, 0, 2))
        elif(self.comando_recebido == 3):
            for i in range(particle_number):
                p2.append(lista_particula.move(0.0, -0.5, 2))
        elif(self.comando_recebido == 4):
            for i in range(particle_number):
                p2.append(lista_particula.move(0.0, 0.5, 2))  
        return p2

    def resampling(self, lista_particulas, particle_number, ponto_recebido):
        # measurement update
        w = []
        for i in range(particle_number):
            w.append(lista_particulas[i].measurement_prob(ponto_recebido))
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
            p3.append(lista_particulas[index])

        return p3
    
    
    #comandos para enviar para o robo
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
        self.process_next_action()
        self.movement_in_progress = False

    def move_forward(self):         
        self.get_logger().info(f'Movendo o robo para frente')
        self.velocity_sender(1.0, 0.0)        
        self.schedule_action(self.stop, self.tal)
        self.movement_in_progress = True

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
        # Remove os pontos antigos
        delete_marker = MarkerArray()
        delete_marker.action = MarkerArray.DELETEALL  
        self.publisher_filtro.publish(delete_marker)         
        
        
        
      

def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Int32
from mapa import Mapa
import math

class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.particle_filter_subscriber = self.create_subscription(Pose, '/filter_final_pose', self.filter_callback, 10)
        self.simulation_subscriber = self.create_subscription(Bool, '/simulation_status', self.simulation_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_publisher = self.create_publisher(Int32, 'robot_commands', 10)
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
        self.grade = None  
        self.simulation_status = False
        self.movement_in_progress = False
        self.get_logger().info('Planner inicializado. Enviando comandos para o Navigation...')        
        self.mapa = Mapa()

    def publish_command(self, comando):
        msg = Int32()
        msg.data = comando
        self.get_logger().info(f'Publishing command: {self.commands[msg.data]}')
        self.command_publisher.publish(msg)
        
    def simulation_callback(self, msg):        
        self.simulation_status = msg.data     
        self.stop()           

    #obtem o ponto final para determinar onde o robo deve ir
    def filter_callback(self, msg):        
        theta = 2 * math.acos(msg.orientation.z)
        self.ponto_final = [msg.position.x, msg.position.y, theta]  
        self.grade = self.mapa.obter_cor_regiao(self.ponto_final[0], self.ponto_final[1])

        if(self.simulation_status):
            
            print(self.grade)

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
        self.movement_in_progress = False  
        self.get_logger().info(f'Robo parado')
        self.velocity_sender(0.0, 0.0)
        self.process_next_action()
        self.publish_command(0)

    def move_forward(self):         
        self.movement_in_progress = True 
        self.get_logger().info(f'Movendo o robo para frente')
        self.velocity_sender(1.0, 0.0)        
        self.schedule_action(self.stop, self.tal)
        self.publish_command(1)

    def move_backward(self):
        self.movement_in_progress = True
        self.get_logger().info(f'Movendo o robo para tras')
        self.velocity_sender(-1.0, 0.0)
        self.schedule_action(self.stop, self.tal)
        self.publish_command(2)    

    def rotate_clockwise(self):
        self.movement_in_progress = True
        self.get_logger().info(f'Rotacionando no sentido horario')
        self.velocity_sender(0.0, -0.5)
        self.schedule_action(self.stop, self.rotate_duration)
        self.publish_command(3)

    def rotate_counter_clockwise(self):
        self.movement_in_progress = True
        self.get_logger().info(f'Rotacionando no sentido anti-horario')
        self.velocity_sender(0.0, 0.5)
        self.schedule_action(self.stop, self.rotate_duration)
        self.publish_command(4)

    def start_movement_sequence(self):        
        for i in range(5):
            #self.add_action_to_queue(self.stop)        
            self.add_action_to_queue(self.move_forward)
        
        
      

def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

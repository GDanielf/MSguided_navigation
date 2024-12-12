#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Twist
from mapa import Mapa
import math

class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.particle_filter_subscriber = self.create_subscription(Pose, '/filter_final_pose', self.filter_callback, 10)
        self.simulation_subscriber = self.create_subscription(Bool, '/simulation_status', self.simulation_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_movement_status_publisher = self.create_publisher(Bool, '/robot_movement_status', 10)
        self.stop_duration = 2.0
        self.rotate_duration = 9
        self.tal = 5.0 
        self.current_timer = None
        self.action_queue = []  # Fila de ações a serem executadas
        #retorno do filtro de particulas
        self.point_final = [0.0, 0.0, 0.0]     
        self.robot_status = False
        self.movement_in_progress = False
        self.get_logger().info('Planner inicializado. Enviando comandos para o Navigation...')        
        self.mapa = Mapa()
        
    def simulation_callback(self, msg):
        if(msg.data):            
            if not self.robot_status:
                #acabei de dar play
                #fica parado por um tempo e depois comeca a andar
                self.robot_status = True
                self.start_movement_sequence()                           
        elif(not msg.data and self.robot_status):
            self.create_timer(self.stop_duration, self.stop)
            self.get_logger().info(f'Simulacao em Pause')
        elif(msg.data and self.robot_status):
            self.start_movement_sequence() 
            self.get_logger().info(f'Simulacao em Play')  

    #obtem o ponto final para determinar onde o robo deve ir
    def filter_callback(self, msg):
        theta = 2 * math.acos(msg.orientation.z)
        self.point_final = [msg.position.x, msg.position.y, theta]   

    def is_robot_moving(self, value):
        status_msg = Bool()
        status_msg.data = value
        self.robot_movement_status_publisher.publish(status_msg)  
        self.get_logger().info(f'robot moving: {status_msg.data}') 

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
        self.is_robot_moving(False)  
        self.get_logger().info(f'Robo parado')
        self.velocity_sender(0.0, 0.0)
        self.process_next_action()

    def move_forward(self):         
        self.movement_in_progress = True  
        self.is_robot_moving(True)  
        self.get_logger().info(f'Movendo o robo para frente')
        self.velocity_sender(1.0, 0.0)        
        self.schedule_action(self.stop, self.tal)

    def move_backward(self):
        self.movement_in_progress = True
        self.is_robot_moving(True)
        self.get_logger().info(f'Movendo o robo para tras')
        self.velocity_sender(-1.0, 0.0)
        self.schedule_action(self.stop, self.tal)

    def rotate_counter_clockwise(self):
        self.movement_in_progress = True
        self.is_robot_moving(True)
        self.get_logger().info(f'Rotacionando no sentido anti-horario')
        self.velocity_sender(0.0, 0.5)
        self.schedule_action(self.stop, self.rotate_duration)

    def rotate_clockwise(self):
        self.movement_in_progress = True
        self.is_robot_moving(True)
        self.get_logger().info(f'Rotacionando no sentido horario')
        self.velocity_sender(0.0, -0.5)
        self.schedule_action(self.stop, self.rotate_duration)

    def start_movement_sequence(self):        
        self.add_action_to_queue(self.stop)        
        self.add_action_to_queue(self.move_forward)
        
        
      

def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

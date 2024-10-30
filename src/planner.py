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
        self.tal = 5.0 
        self.current_timer = None
        #retorno do filtro de particulas
        self.point_final = [0.0, 0.0, 0.0]     
        self.robot_status = False  
        self.get_logger().info('Planner inicializado. Enviando comandos para o Navigation...')        
        self.mapa = Mapa()

    def is_robot_moving(self, value):
        status_msg = Bool()
        status_msg.data = value
        self.robot_movement_status_publisher.publish(status_msg)  
        self.get_logger().info(f'robot moving: {status_msg.data}') 

    def velocity_sender(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = linear  # Mover para frente com velocidade linear
        cmd.angular.z = angular
        self.cmd_vel_publisher.publish(cmd)        
        if linear == 1.0:
            self.get_logger().info('Movendo para frente...')  

    def stop(self):
        self.get_logger().info(f'Robo parado')
        self.velocity_sender(0.0, 0.0)
        self.is_robot_moving(False)
        # Cancela o timer atual para evitar múltiplas execuções
        if self.current_timer:
            self.current_timer.cancel()

        # Cria um novo timer para retomar o movimento após o tempo de parada
        self.current_timer = self.create_timer(self.tal, self.move_forward)


    def move_forward(self):
        self.get_logger().info(f'Movendo o robo para frente')
        self.velocity_sender(1.0, 0.0)
        self.is_robot_moving(True)
        # Cancela o timer atual para evitar múltiplas execuções
        if self.current_timer:
            self.current_timer.cancel()

        # Cria um novo timer para parar o movimento após o tempo de movimento
        self.current_timer = self.create_timer(self.stop_duration, self.stop)


    def move_backward(self):
        self.get_logger().info(f'Movendo o robo para tras')
        self.velocity_sender(-1.0, 0.0)
        self.is_robot_moving(True)
        # Cancela o timer atual para evitar múltiplas execuções
        if self.current_timer:
            self.current_timer.cancel()

        # Cria um novo timer para parar o movimento após o tempo de movimento
        self.current_timer = self.create_timer(self.stop_duration, self.stop)

    def rotate_clockwise(self):
        self.velocity_sender(0.0, 0.5)

    def rotate_counter_clockwise(self):
        self.velocity_sender(0.0, -0.5)

    def start_movement_sequence(self):
        # Começa o movimento parando primeiro (pode ajustar conforme necessário)
        self.stop()
        # Inicia o primeiro timer de movimento após o tempo de parada
        self.current_timer = self.create_timer(self.stop_duration, self.move_forward)

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
            self.get_logger().info(f'Simulacao em Play, pare o robo') 
    
    #obtem o ponto final para determinar onde o robo deve ir
    def filter_callback(self, msg):
        theta = 2 * math.acos(msg.orientation.z)
        self.point_final = [msg.position.x, msg.position.y, theta]         

def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

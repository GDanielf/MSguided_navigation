#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Twist, Quaternion
import numpy as np

class Teste(Node):
    def __init__(self):
        super().__init__('teste')
        self.publisher_ponto_est = self.create_publisher(Marker, 'topic_pose_est', 10)
        self.publisher_angulo_teste = self.create_publisher(Float64, '/topico_teste', 10)
        self.timer = self.create_timer(1.0, self.publish_ponto_pose_estimada)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.angulo_yaw = (np.pi / 4) 

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]
    
    def publish_message(self):
        msg = Float64()
        msg.data = self.angulo_yaw
        self.publisher_angulo_teste.publish(msg)
        self.get_logger().info(f'Publicando: "{msg.data}"')

    def publish_ponto_pose_estimada(self):            
        quaternion_euler = self.euler_to_quaternion(0, 0, self.angulo_yaw)
        
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
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
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
    
def main(args=None):
    rclpy.init(args=args)
    node = Teste()  # Inicializa a classe
    try:
        rclpy.spin(node)  # Mantém o node rodando
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
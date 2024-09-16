#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from guided_navigation.msg import ImagesAngles
import numpy as np
import matplotlib.pyplot as plt

class Triangulation(Node):
    def __init__(self):
        super().__init__('triangulation')

        #Posicoes e rotacoes das cameras no mundo
        self.camera0_pos = np.array([9.7764, -7.4141])
        self.camera0_rot = np.array([-0.170718217990764, 0.0742291662230543, 0.90103212642953279])
        self.camera1_pos = np.array([-9.80918, -7.36591])
        self.camera1_rot = np.array([-0.053561496744893217, 0.17391950331266406, 0.28941120545221088])
        self.camera2_pos = np.array([9.9253, 7.43722])
        self.camera2_rot = np.array([0.15519983539502044, 0.050682763922531189, -0.937840950144712])
        self.camera3_pos = np.array([-9.9422, 7.4141])
        self.camera3_rot = np.array([0.064117097027054454, 0.14912927654399707, -0.38974484286365219])

        self.subscription = self.create_subscription(
            ImagesAngles,
            '/image_angles',
            self.triangulation_callback,
            10
        )

        # Desativar mensagens de retorno de chamada não utilizadas
        self.subscription

    def yaw_rotation(self,x,y,z):
        w = np.sqrt(1 - x**2 - y**2 - z**2)
        return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    

    def triangulation_callback(self, msg):
        camera_position = [self.camera0_pos, self.camera1_pos, self.camera2_pos, self.camera3_pos]
        camera_rotations = [self.yaw_rotation(self.camera0_rot[0], self.camera0_rot[1], self.camera0_rot[2]), 
                     self.yaw_rotation(self.camera1_rot[0], self.camera1_rot[1], self.camera1_rot[2]),
                     self.yaw_rotation(self.camera2_rot[0], self.camera2_rot[1], self.camera2_rot[2]),
                     self.yaw_rotation(self.camera3_rot[0], self.camera3_rot[1], self.camera3_rot[2])]

        fig, ax = plt.subplots()
        print('Angle image 0: ', msg.angle_image_0, 'Angle image 1: ', msg.angle_image_1, 'Angle image 2: ', msg.angle_image_2, 'Angle image 3: ', msg.angle_image_3)

        # Plotar cada ponto e vetor da camera
        for pos, angle in zip(camera_position, camera_rotations):
            # Calcular o vetor unitário
            unit_vector = np.array([2*np.cos(angle), 2*np.sin(angle)])
            
            # Adicionar o vetor unitário à posição
            end_pos = pos + unit_vector

            # Plotar a linha do vetor unitário
            ax.quiver(pos[0], pos[1], unit_vector[0], unit_vector[1], angles='xy', scale_units='xy', scale=1, color='r')

            # Plotar a posição
            ax.scatter(pos[0], pos[1], color='b')
        
        camera_angles = [camera_rotations[0] - msg.angle_image_0,camera_rotations[1] - msg.angle_image_1, 
                         camera_rotations[2] - msg.angle_image_2, camera_rotations[3] - msg.angle_image_3]

        # Plotar cada ponto e vetor das imagens
        for pos, angle in zip(camera_position, camera_angles):
            # Calcular o vetor unitário
            unit_vector = np.array([3*np.cos(angle), 3*np.sin(angle)])
            
            # Adicionar o vetor unitário à posição
            end_pos = pos + unit_vector

            # Plotar a linha do vetor unitário
            ax.quiver(pos[0], pos[1], unit_vector[0], unit_vector[1], angles='xy', scale_units='xy', scale=1, color='g')

            # Plotar a posição
            ax.scatter(pos[0], pos[1], color='b')
        
        # Configurar o gráfico
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.legend()
        ax.grid(True)
        plt.title('Posições e Vetores')
        plt.show()




def main(args=None):
    rclpy.init(args=args)
    node = Triangulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()

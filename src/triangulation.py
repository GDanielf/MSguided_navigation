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
        self.camera1_pos = np.array([9.7764, -7.4141])
        self.camera1_rot = np.array([0.90103212642953279])
        self.camera2_pos = np.array([-9.80918, -7.36591])
        self.camera2_rot = np.array([0.28941120545221088])
        self.camera3_pos = np.array([9.9253, 7.43722])
        self.camera3_rot = np.array([-0.937840950144712])
        self.camera4_pos = np.array([-9.9422, 7.4141])
        self.camera4_rot = np.array([-0.38974484286365219])

        self.subscription = self.create_subscription(
            ImagesAngles,
            '/image_angles',
            self.triangulation_callback,
            10
        )

        # Desativar mensagens de retorno de chamada não utilizadas
        self.subscription


    def triangulation_callback(self, msg):
        camera_angles = [msg.angle_image_1, msg.angle_image_2, msg.angle_image_3, msg.angle_image_4]
        positions = [self.camera1_pos, self.camera2_pos, self.camera3_pos, self.camera4_pos]
        rotations = [self.camera1_rot, self.camera2_rot, self.camera3_rot, self.camera4_rot]

        fig, ax = plt.subplots()

        # Plotar cada ponto e vetor da camera
        for pos, angle in zip(positions, rotations):
            # Calcular o vetor unitário
            unit_vector = np.array([np.cos(angle), np.sin(angle)])
            
            # Adicionar o vetor unitário à posição
            end_pos = pos + unit_vector

            # Plotar a linha do vetor unitário
            ax.quiver(pos[0], pos[1], unit_vector[0], unit_vector[1], angles='xy', scale_units='xy', scale=1, color='r')

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
    #cv2.destroyAllWindows()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()

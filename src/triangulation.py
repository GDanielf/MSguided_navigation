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
    
    def intersecao_retas(self, A1, B1, C1, A2, B2, C2):
            A = np.array([[A1, B1],
                        [A2, B2]])
            C = np.array([-C1, -C2])
            return np.linalg.solve(A, C)        
 

    def triangulation_callback(self, msg):
        camera_position = [self.camera0_pos, self.camera1_pos, self.camera2_pos, self.camera3_pos]
        camera_rotations = [self.yaw_rotation(self.camera0_rot[0], self.camera0_rot[1], self.camera0_rot[2]), 
                     self.yaw_rotation(self.camera1_rot[0], self.camera1_rot[1], self.camera1_rot[2]),
                     self.yaw_rotation(self.camera2_rot[0], self.camera2_rot[1], self.camera2_rot[2]),
                     self.yaw_rotation(self.camera3_rot[0], self.camera3_rot[1], self.camera3_rot[2])]

        fig, ax = plt.subplots()
        #print('Angle image 0: ', msg.angle_image_0, 'Angle image 1: ', msg.angle_image_1, 'Angle image 2: ', msg.angle_image_2, 'Angle image 3: ', msg.angle_image_3)

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
        
        image_angles = [camera_rotations[0] - msg.angle_image_0,camera_rotations[1] - msg.angle_image_1, 
                         camera_rotations[2] - msg.angle_image_2, camera_rotations[3] - msg.angle_image_3]

        # Plotar cada ponto e vetor das imagens
        for pos, angle in zip(camera_position, image_angles):
            # Calcular o vetor unitário
            unit_vector = np.array([30*np.cos(angle), 30*np.sin(angle)])
            
            # Adicionar o vetor unitário à posição
            end_pos = pos + unit_vector

            # Plotar a linha do vetor unitário
            ax.quiver(pos[0], pos[1], unit_vector[0], unit_vector[1], angles='xy', scale_units='xy', scale=1, color='g')

            # Plotar a posição
            ax.scatter(pos[0], pos[1], color='b')

        # Cada reta pode ser representada pela equação da forma geral: Ax+By+C=0
        # A equação da reta em termos de um ponto (x0,y0)(x0​,y0​) e o ângulo θθ seria: y−y0=tan⁡(θ)(x−x0)
        # Ou: tan(θ)x−y+(y0​−tan(θ)x0​)=0
        # A= tan(θ), B=−1, e C = y0 − tan⁡(θ)*x0
        # Encontrando as interseccoes:
        B = -1
        theta_0 = image_angles[0]
        A0 = np.tan(theta_0)
        C0 = camera_position[0][1] - A0*camera_position[0][0]
        theta_1 = image_angles[1]
        A1 = np.tan(theta_1)
        C1 = camera_position[1][1] - A1*camera_position[1][0]
        theta_2 = image_angles[2]
        A2 = np.tan(theta_2)
        C2 = camera_position[2][1] - A2*camera_position[2][0]
        theta_3 = image_angles[3]
        A3 = np.tan(theta_3)
        C3 = camera_position[3][1] - A3*camera_position[3][0]  

        xlimit = [-18, 18]
        ylimit = [-8, 8]
        retas = [(A0, B, C0), (A1, B, C1), (A2, B, C2), (A3, B, C3)]
        pontos_interseccao = []
        for i in range(len(retas)):
            for j in range(i + 1, len(retas)):
                A1, B1, C1 = retas[i]
                A2, B2, C2 = retas[j]
                ponto = self.intersecao_retas(A1, B1, C1, A2, B2, C2)
                if(xlimit[0] < ponto[0] and xlimit[1] > ponto[0] and ylimit[0] < ponto[1] and ylimit[1] > ponto[1]):
                    pontos_interseccao.append(ponto)  

        # Plotar os pontos de interseccao
        for ponton in pontos_interseccao:           

            # Plotar a posição
            ax.scatter(ponton[0], ponton[1], color='y')   
        
        # centro
        ax.scatter(0, 0, color='black')   
        # Configurar o gráfico
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_xlim(xlimit)
        ax.set_ylim(ylimit)
        ax.legend()
        ax.grid(True)
        plt.title('Posições e Vetores')
        plt.show()  

        print(f"Ponto de interseção : {pontos_interseccao}")
        
def main(args=None):
    rclpy.init(args=args)
    node = Triangulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()

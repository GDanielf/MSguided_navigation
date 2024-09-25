#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from guided_navigation.msg import ImagesAngles
import numpy as np
import matplotlib.pyplot as plt
import math
from mapa import Mapa

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
        self.hfov = 1.0469999999999999
        self.camera_position_vec = 2
        self.image_position_vec = 30        
        self.xlimit = [-250, 250]
        self.ylimit = [-150, 150]
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

    def verificar_direcao(self, ponto, origem, direcao):
        vetor_intersecao = ponto - origem
        produto_escalar = np.dot(vetor_intersecao, direcao)
        return produto_escalar > 0   
    
    def baricentro(self, lista_pontos_interseccao):
        x = 0
        y = 0
        if(lista_pontos_interseccao != []):
            for i in range(len(lista_pontos_interseccao)):
                x = x + lista_pontos_interseccao[i][0]
                y = y + lista_pontos_interseccao[i][1]        
            x = x/(len(lista_pontos_interseccao))
            y = y/(len(lista_pontos_interseccao))
        return x,y
    
    def ponto_intersec_valid(self, angle_result, ponto_inicial, ponto_intersec):
        result = False
        if angle_result >= 0 and ponto_inicial != [] and ponto_intersec != []:
            result = (ponto_intersec[0] >= ponto_inicial[0] and ponto_intersec[1] >= ponto_inicial[1] or 
                      ponto_intersec[0] <= ponto_inicial[0] and ponto_intersec[1] >= ponto_inicial[1])
                   
        elif angle_result < 0 and ponto_inicial != [] and ponto_intersec != []:
            result = (ponto_intersec[0] < ponto_inicial[0] and ponto_intersec[1] < ponto_inicial[1] or 
                    (ponto_intersec[0] >= ponto_inicial[0] and ponto_intersec[1] < ponto_inicial[1]))

        return result

         
    def triangulation_callback(self, msg):
        mapa = Mapa()
        camera_position = [self.camera0_pos, self.camera1_pos, self.camera2_pos, self.camera3_pos]
        camera_rotations = [self.yaw_rotation(self.camera0_rot[0], self.camera0_rot[1], self.camera0_rot[2]), 
                     self.yaw_rotation(self.camera1_rot[0], self.camera1_rot[1], self.camera1_rot[2]),
                     self.yaw_rotation(self.camera2_rot[0], self.camera2_rot[1], self.camera2_rot[2]),
                     self.yaw_rotation(self.camera3_rot[0], self.camera3_rot[1], self.camera3_rot[2])]

        
        #print('Angle image 0: ', msg.angle_image_0, 'Angle image 1: ', msg.angle_image_1, 'Angle image 2: ', msg.angle_image_2, 'Angle image 3: ', msg.angle_image_3)
        labels = ['Cam 0', 'Cam 1', 'Cam 2', 'Cam 3']
        cameras = [self.camera0_pos, self.camera1_pos, self.camera2_pos, self.camera3_pos]
        tex_pos_01 = 2
        tex_pos_23 = 1
        # Plotar cada ponto e vetor da camera
        fig, ax = plt.subplots()
        for pos, angle in zip(camera_position, camera_rotations):
            # Calcular o vetor unitário
            unit_vector = np.array([self.camera_position_vec*np.cos(angle), self.camera_position_vec*np.sin(angle)])
            
            # Adicionar o vetor unitário à posição
            end_pos = pos + unit_vector

            # Plotar a linha do vetor unitário
            ax.quiver(pos[0], pos[1], unit_vector[0], unit_vector[1], angles='xy', scale_units='xy', scale=1, color= 'r')

            # Plotar a posição
            ax.scatter(pos[0], pos[1], color='b')

        plt.text(cameras[0][0] - tex_pos_01, cameras[0][1] - tex_pos_01, labels[0], fontsize=12)
        plt.text(cameras[1][0] - tex_pos_01, cameras[1][1] - tex_pos_01, labels[1], fontsize=12)
        plt.text(cameras[2][0] - tex_pos_23, cameras[2][1] + tex_pos_23, labels[2], fontsize=12)
        plt.text(cameras[3][0] - tex_pos_23, cameras[3][1] + tex_pos_23, labels[3], fontsize=12)
            

        up_hfov = 0
        down_hfov = 0
        hfov_limit = []
        for i in range(len(camera_rotations)):
            up_hfov = camera_rotations[i] + self.hfov
            down_hfov = camera_rotations[i] - self.hfov
            hfov_limit.append([up_hfov, down_hfov])
        #print('hfov limite: ', hfov_limit)
        hfov_limit[0] = [3.1416, 1.57]
        hfov_limit[1] = [1.57, 0]
        hfov_limit[2] = [-1.57, -3.1416]
        hfov_limit[3] = [0, -1.57]

        #plotando os limites de hfov
        for pos, angle in zip(camera_position, hfov_limit):
            # Calcular o vetor unitário
            unit_vector = np.array([self.camera_position_vec*np.cos(angle[0]), self.camera_position_vec*np.sin(angle[0])])
            unit_vector_1 = np.array([self.camera_position_vec*np.cos(angle[1]), self.camera_position_vec*np.sin(angle[1])])
            
            # Adicionar o vetor unitário à posição
            end_pos = pos + unit_vector

            # Plotar a linha do vetor unitário
            ax.quiver(pos[0], pos[1], unit_vector[0], unit_vector[1], angles='xy', scale_units='xy', scale=1, color= 'k')
            ax.quiver(pos[0], pos[1], unit_vector_1[0], unit_vector_1[1], angles='xy', scale_units='xy', scale=1, color= 'k')

            # Plotar a posição
            ax.scatter(pos[0], pos[1], color='b') 
        
        image_angles = [msg.angle_image_0, msg.angle_image_1, msg.angle_image_2, msg.angle_image_3]

        image_angles_res = []

        for i in range(len(image_angles)):
            if math.isnan(image_angles[i]):
                image_angles_res.append(float('nan'))
            else:
                image_angles_res.append(np.clip(camera_rotations[i] - image_angles[i], hfov_limit[i][1], hfov_limit[i][0]))

        # Cada reta pode ser representada pela equação da forma geral: Ax+By+C=0
        # A equação da reta em termos de um ponto (x0,y0)(x0​,y0​) e o ângulo θθ seria: y−y0=tan⁡(θ)(x−x0)
        # Ou: tan(θ)x−y+(y0​−tan(θ)x0​)=0
        # A= tan(θ), B=−1, e C = y0 − tan⁡(θ)*x0
        # Encontrando as interseccoes:
        B = -1 
        Ax = []
        Cx = []  

        #teste
        #image_angles_res = np.array([1.0469999999999999, -1.0469999999999999, 1.0469999999999999, -1.0469999999999999])

        for i in range(len(image_angles_res)):
            if not math.isnan(image_angles_res[i]):
                Ax.append(np.tan(image_angles_res[i]))
                Cx.append(camera_position[i][1] - np.tan(image_angles_res[i])*camera_position[i][0])
        
        retas = [(Ax[i], B, Cx[i]) for i in range(len(Ax))]
        
        #print('ang: ', image_angles)
        #print('ang tratado: ', image_angles_res)
        #print('pos tratado: ', camera_rotations)
        # Plotar cada ponto e vetor das imagens
        for pos, angle in zip(camera_position, image_angles_res):
            if not math.isnan(angle):
                # Calcular o vetor unitário
                unit_vector = np.array([self.image_position_vec*np.cos(angle), self.image_position_vec*np.sin(angle)])
                
                # Adicionar o vetor unitário à posição
                end_pos = pos + unit_vector

                # Plotar a linha do vetor unitário
                ax.quiver(pos[0], pos[1], unit_vector[0], unit_vector[1], angles='xy', scale_units='xy', scale=1, color= 'g')

                # Plotar a posição
                ax.scatter(pos[0], pos[1], color='b')         
        
        pontos_interseccao = []
        ponto = []
        print(retas)
        for i in range(len(retas)):
            for j in range(i + 1, len(retas)):
                if not math.isnan(retas[i][0]) or not math.isnan(retas[i][2]): #se Ax nao for 'nan'
                    A1, B1, C1 = retas[i]
                    A2, B2, C2 = retas[j]
                    ponto.append(self.intersecao_retas(A1, B1, C1, A2, B2, C2))
                    #pontos_interseccao.append(ponto)  

        # Plotar os pontos de interseccao
        for j in ponto:       
            # Plotar a posição
            ax.scatter(j[0], j[1], color='y')  

        #plotando
        
        bar_x, bar_y = self.baricentro(ponto)
        print('pontos de intersec: ', ponto)
        print('baricentro: ', [bar_x, bar_y])
        ax.scatter(float(bar_x), float(bar_y), color='c')
        mapa.desenhar_mapa(ax)
        # centro
        ax.scatter(0, 0, color='black')          
        # Configurar o gráfico
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_xlim(self.xlimit)
        ax.set_ylim(self.ylimit)
        ax.legend()
        ax.grid(True)
        plt.title('Posições e Vetores')
        plt.show()  
        #pontos_insterseccao = pos_list()
        #print(f"Ponto de interseção : {pontos_interseccao}")
        for i in range(len(camera_rotations)):
            print('Limites Camera ', i, hfov_limit[i][0], camera_rotations[i], hfov_limit[i][1])
        
def main(args=None):
    rclpy.init(args=args)
    node = Triangulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from guided_navigation.msg import ImagesAngles
from guided_navigation.msg import PoseEstimate
import numpy as np
import matplotlib.pyplot as plt
import math
from mapa import Mapa

class Triangulation(Node):
    def __init__(self):
        super().__init__('triangulation')
        self.mapa = Mapa()
        #Posicoes e rotacoes das cameras no mundo
        self.camera0_pos = np.array([9.9843, -7.4687])
        self.camera0_rot = np.array([-0.20651563011976296, 0.0693197799957628, 0.92525149065264534])
        self.camera1_pos = np.array([-9.9843, -7.4687])
        self.camera1_rot = np.array([-0.068773030063436352, 0.20669834957811681, 0.30812364442338763])
        self.camera2_pos = np.array([9.9843, 7.4687])
        self.camera2_rot = np.array([0.20659175692043724, 0.069092570909980416, -0.925592561378196])
        self.camera3_pos = np.array([-9.9843, 7.4687])
        self.camera3_rot = np.array([0.069434111535624884, 0.20647721796285928, -0.3110854861843747])
        self.camera4_pos = np.array([-3.8126, -2.3696])
        self.camera4_rot = np.array([-0.18549626982220258, 0.031907839433870766, 0.96791159620266265])
        self.camera5_pos = np.array([-3.8126, 2.3696])
        self.camera5_rot = np.array([0.18549626982220258, 0.031907839433870766, -0.96791159620266265])
        self.camera6_pos = np.array([3.8126, 2.3696])
        self.camera6_rot = np.array([0.031908520799500469, 0.18549615261691255, -0.16649729576294525])
        self.camera7_pos = np.array([3.8126, -2.3696])
        self.camera7_rot = np.array([-0.031908520799500469, 0.18549615261691255, 0.16649729576294525])
        self.hfov = 1.0469999999999999
        self.camera_position_vec = 2
        self.image_position_vec = 30        
        self.xlimit = [-25, 25]
        self.ylimit = [-15, 15]

        #subscribeer do robot status
        self.robot_status_subscription = self.create_subscription(
            Bool,
            '/robot_status',
            self.robot_status_callback,
            10
        )
        #subscriber dos angulos das cameras
        self.image_angle_subscription = self.create_subscription(
            ImagesAngles,
            '/image_angles',
            self.triangulation_callback,
            10
        )

        # Desativar mensagens de retorno de chamada não utilizadas
        self.image_angle_subscription
        self.robot_status_subscription

        self.publisher = self.create_publisher(PoseEstimate, 'pose_estimate', 10)
        self.pose_x = 0
        self.pose_y = 0

    def publish_pose_estimate(self):
        # Cria a mensagem de pose estimada e publica
        msg = PoseEstimate()
        msg.x = float(self.pose_x)  # Aqui você insere o valor de x
        msg.y = float(self.pose_y)  # Aqui você insere o valor de y
        self.publisher.publish(msg)

    def robot_status_callback(self, msg):
        # Verifica se o robô está parado
        if not msg.data:  # msg.data == False indica que o robô parou
            self.publish_pose_estimate()

    def yaw_rotation(self,x,y,z):
        w = np.sqrt(1 - x**2 - y**2 - z**2)
        return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    
    def compara_ponto(self, camera_list, camera_desejada, ponto):
        a = round(ponto[0],2)
        b = round(ponto[1],2)
        if camera_desejada == 0:
            return (a <= round(camera_list[0][0],2) and b >= round(camera_list[0][1],2))
        elif camera_desejada == 1:
            return (a >= round(camera_list[1][0],2) and b >= round(camera_list[1][1],2))
        elif camera_desejada == 2:
            return (a <= round(camera_list[2][0],2) and b <= round(camera_list[2][1],2))
        elif camera_desejada == 3:
            return (a >= round(camera_list[3][0],2) and b <= round(camera_list[3][1],2))
        elif camera_desejada == 4:
            return (a <= round(camera_list[4][0],2) and b >= round(camera_list[4][1],2))
        elif camera_desejada == 5:
            return (a <= round(camera_list[5][0],2) and b <= round(camera_list[5][1],2))
        elif camera_desejada == 6:
            return (a >= round(camera_list[6][0],2) and b <= round(camera_list[6][1],2))
        elif camera_desejada == 7:
            return (a >= round(camera_list[7][0],2) and b >= round(camera_list[7][1],2))
    
    
    def intersecao_retas(self, A1, B1, C1, A2, B2, C2, camera_list, camera1, camera2):
        try:
            # Montagem das matrizes
            A = np.array([[A1, B1],
                        [A2, B2]])
            C = np.array([-C1, -C2])

            # Verificar se as retas são paralelas
            det = np.linalg.det(A)
            if det == 0:
                #print("As retas são paralelas ou coincidentes. Não há interseção única.")
                return None

            # Resolver a equação para encontrar a interseção
            ponto = np.linalg.solve(A, C)
            #print(ponto)
            if(self.compara_ponto(camera_list, camera1, ponto) and self.compara_ponto(camera_list, camera2, ponto)):
                return ponto
            else:
                return None
        except np.linalg.LinAlgError:
            #print("Erro: Matriz singular. Não é possível calcular a interseção.")
            return None

    def verificar_direcao(self, ponto, origem, direcao):
        produto_escalar = 0
        if not math.isnan(direcao[0]):
            vetor_intersecao = ponto - origem
            produto_escalar = np.dot(vetor_intersecao, direcao)   
            #print(produto_escalar > 0)         
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
    
    def plotting_all(self, camera_position, camera_rotations, hfov_limit, image_angles_res, pontos_interseccao, bar_x, bar_y):
        fig, ax = plt.subplots()
        # Plotar cada ponto e vetor da camera        
        for pos, angle in zip(camera_position, camera_rotations):
            # Calcular o vetor unitário
            unit_vector = np.array([self.camera_position_vec*np.cos(angle), self.camera_position_vec*np.sin(angle)])
            
            # Adicionar o vetor unitário à posição
            end_pos = pos + unit_vector

            # Plotar a linha do vetor unitário
            ax.quiver(pos[0], pos[1], unit_vector[0], unit_vector[1], angles='xy', scale_units='xy', scale=1, color= 'r')

            # Plotar a posição
            ax.scatter(pos[0], pos[1], color='b')
        
        labels = ['Cam 0', 'Cam 1', 'Cam 2', 'Cam 3', 'Cam 4', 'Cam 5', 'Cam 6', 'Cam 7']
        tex_pos_01 = 2
        tex_pos_23 = 1
        tex_pos_45 = 0      

        plt.text(camera_position[0][0] - tex_pos_01, camera_position[0][1] - tex_pos_01, labels[0], fontsize=12)
        plt.text(camera_position[1][0] - tex_pos_01, camera_position[1][1] - tex_pos_01, labels[1], fontsize=12)
        plt.text(camera_position[2][0] - tex_pos_23, camera_position[2][1] + tex_pos_23, labels[2], fontsize=12)
        plt.text(camera_position[3][0] - tex_pos_23, camera_position[3][1] + tex_pos_23, labels[3], fontsize=12)
        plt.text(camera_position[4][0] - tex_pos_45, camera_position[4][1] + tex_pos_45, labels[4], fontsize=12)
        plt.text(camera_position[5][0] - tex_pos_45, camera_position[5][1] + tex_pos_45, labels[5], fontsize=12)
        plt.text(camera_position[6][0] - tex_pos_45, camera_position[6][1] + tex_pos_45, labels[6], fontsize=12)
        plt.text(camera_position[7][0] - tex_pos_45, camera_position[7][1] + tex_pos_45, labels[7], fontsize=12)

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

            # Plotar cada ponto e vetor das imagens
        for pos, angle in zip(camera_position, image_angles_res):
            if not math.isnan(angle):
                # Calcular o vetor unitário
                unit_vector = np.array([np.cos(angle), np.sin(angle)])
                final_vector = np.array([self.image_position_vec*np.cos(angle), self.image_position_vec*np.sin(angle)])

                # Plotar a linha do vetor unitário
                ax.quiver(pos[0], pos[1], final_vector[0], final_vector[1], angles='xy', scale_units='xy', scale=1, color= 'g')

                # Plotar a posição
                ax.scatter(pos[0], pos[1], color='b')   


        # Plotar os pontos de interseccao
        for j in pontos_interseccao:       
            # Plotar a posição
            ax.scatter(j[0], j[1], color='y')  


        ax.scatter(float(bar_x), float(bar_y), color='c')        
        self.mapa.desenhar_mapa(ax)
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
        
    def triangulation_callback(self, msg):        
        camera_position = [self.camera0_pos, self.camera1_pos, self.camera2_pos, self.camera3_pos,
                           self.camera4_pos, self.camera5_pos, self.camera6_pos, self.camera7_pos]
        camera_rotations = [self.yaw_rotation(self.camera0_rot[0], self.camera0_rot[1], self.camera0_rot[2]), 
                     self.yaw_rotation(self.camera1_rot[0], self.camera1_rot[1], self.camera1_rot[2]),
                     self.yaw_rotation(self.camera2_rot[0], self.camera2_rot[1], self.camera2_rot[2]),
                     self.yaw_rotation(self.camera3_rot[0], self.camera3_rot[1], self.camera3_rot[2]),
                     self.yaw_rotation(self.camera4_rot[0], self.camera4_rot[1], self.camera4_rot[2]),
                     self.yaw_rotation(self.camera5_rot[0], self.camera5_rot[1], self.camera5_rot[2]),
                     self.yaw_rotation(self.camera6_rot[0], self.camera6_rot[1], self.camera6_rot[2]),
                     self.yaw_rotation(self.camera7_rot[0], self.camera7_rot[1], self.camera7_rot[2])]                
        #print('Angle image 0: ', msg.angle_image_0, 'Angle image 1: ', msg.angle_image_1, 'Angle image 2: ', msg.angle_image_2, 'Angle image 3: ', msg.angle_image_3)     
           
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
        hfov_limit[4] = [3.1416, 1.57]
        hfov_limit[5] = [-1.57, -3.1416]
        hfov_limit[6] = [0, -1.57]
        hfov_limit[7] = [1.57, 0]        
        
        image_angles = [msg.angle_image_0, msg.angle_image_1, msg.angle_image_2, msg.angle_image_3,
                         msg.angle_image_4, msg.angle_image_5, msg.angle_image_6, msg.angle_image_7]

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
            else:
                Ax.append(float('nan'))
                Cx.append(float('nan'))
        
        retas = [(Ax[i], B, Cx[i]) for i in range(len(Ax))]
        #print('retas: ', retas)
        #print('ang: ', image_angles)
        #print('ang tratado: ', image_angles_res)
        #print('pos tratado: ', camera_rotations)       
        
        pontos_interseccao = []
        ponto = []
        #print(len(retas))
        for i in range(len(retas)):
            for j in range(i + 1, len(retas)):                
                A1, B1, C1 = retas[i]
                A2, B2, C2 = retas[j]
                ponto = self.intersecao_retas(A1, B1, C1, A2, B2, C2, camera_position, i, j)
                
                #print('combinacao i j: ', i, j, 'ponto: ', ponto)
                if(ponto is not None and self.mapa.verifica_ponto_dentro(ponto)):
                    pontos_interseccao.append(ponto)

        #plotando
        
        bar_x, bar_y = self.baricentro(pontos_interseccao)
        self.pose_x = float(bar_x)
        self.pose_y = float(bar_y)
        #self.plotting_all(camera_position, camera_rotations, hfov_limit, image_angles_res, pontos_interseccao, bar_x, bar_y)
        #print('pontos de intersec: ', pontos_interseccao)
        #print('baricentro: ', [bar_x, bar_y])
        #pontos_insterseccao = pos_list()
        #print(f"Ponto de interseção : {pontos_interseccao}")
        #for i in range(len(camera_rotations)):
            #print('Limites Camera ', i, hfov_limit[i][0], camera_rotations[i], hfov_limit[i][1])
        
def main(args=None):
    rclpy.init(args=args)
    node = Triangulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()

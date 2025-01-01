#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from guided_navigation.msg import ImagesAngles
from guided_navigation.msg import PoseEstimate
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt
from math import *
from mapa import Mapa

class Triangulation(Node):
    def __init__(self):
        super().__init__('triangulation')
        self.mapa = Mapa()
        #Posicoes e rotacoes das cameras no mundo
        self.camera0_pos = np.array([-9.0, -7.4687])
        self.camera0_rot = np.array([-0.15397426496748834, 0.15409692764667762, 0.68985053625575676])
        self.camera1_pos = np.array([-7.0, -7.4687])
        self.camera1_rot = np.array([-0.15397426496748834, 0.15409692764667762, 0.68985053625575676])
        self.camera2_pos = np.array([-5.0, -7.4687])
        self.camera2_rot = np.array([-0.15397426496748834, 0.15409692764667762, 0.68985053625575676])
        self.camera3_pos = np.array([-3.0, -7.4687])
        self.camera3_rot = np.array([-0.15397426496748834, 0.15409692764667762, 0.68985053625575676])
        self.camera4_pos = np.array([-1.0, -7.4687])
        self.camera4_rot = np.array([-0.15397426496748834, 0.15409692764667762, 0.68985053625575676])
        self.camera5_pos = np.array([1.0, -7.4687])
        self.camera5_rot = np.array([-0.15397426496748834, 0.15409692764667762, 0.68985053625575676])
        self.camera6_pos = np.array([3.0, -7.4687])
        self.camera6_rot = np.array([-0.15397426496748834, 0.15409692764667762, 0.68985053625575676])
        self.camera7_pos = np.array([5.0, -7.4687])
        self.camera7_rot = np.array([-0.15397426496748834, 0.15409692764667762, 0.68985053625575676])
        self.camera8_pos = np.array([7.0, -7.4687])
        self.camera8_rot = np.array([-0.15397426496748834, 0.15409692764667762, 0.68985053625575676])
        self.camera9_pos = np.array([9.0, -7.4687])
        self.camera9_rot = np.array([-0.15397426496748834, 0.15409692764667762, 0.68985053625575676])
        self.camera10_pos = np.array([-8.0, 7.4687])
        self.camera10_rot = np.array([0.15397426496748834, 0.15409692764667762, -0.68985053625575676])
        self.camera11_pos = np.array([-6.0, 7.4687])
        self.camera11_rot = np.array([0.15397426496748834, 0.15409692764667762, -0.68985053625575676])
        self.camera12_pos = np.array([-4.0, 7.4687])
        self.camera12_rot = np.array([0.15397426496748834, 0.15409692764667762, -0.68985053625575676])
        self.camera13_pos = np.array([-2.0, 7.4687])
        self.camera13_rot = np.array([0.15397426496748834, 0.15409692764667762, -0.68985053625575676])
        self.camera14_pos = np.array([0.0, 7.4687])
        self.camera14_rot = np.array([0.15397426496748834, 0.15409692764667762, -0.68985053625575676])
        self.camera15_pos = np.array([2.0, 7.4687])
        self.camera15_rot = np.array([0.15397426496748834, 0.15409692764667762, -0.68985053625575676])
        self.camera16_pos = np.array([4.0, 7.4687])
        self.camera16_rot = np.array([0.15397426496748834, 0.15409692764667762, -0.68985053625575676])
        self.camera17_pos = np.array([6.0, 7.4687])
        self.camera17_rot = np.array([0.15397426496748834, 0.15409692764667762, -0.68985053625575676])
        self.camera18_pos = np.array([8.0, 7.4687])
        self.camera18_rot = np.array([0.15397426496748834, 0.15409692764667762, -0.68985053625575676])
        self.camera19_pos = np.array([9.983, -6.5])
        self.camera19_rot = np.array([-0.21783917758328675, 0.00017347121075593651, 0.97598435365204861])
        self.camera20_pos = np.array([9.983, -4.5])
        self.camera20_rot = np.array([-0.21783917758328675, 0.00017347121075593651, 0.97598435365204861])
        self.camera21_pos = np.array([9.983, -2.5])
        self.camera21_rot = np.array([-0.21783917758328675, 0.00017347121075593651, 0.97598435365204861])
        self.camera22_pos = np.array([9.983, -0.5])
        self.camera22_rot = np.array([-0.21783917758328675, 0.00017347121075593651, 0.97598435365204861])
        self.camera23_pos = np.array([9.983, 1.5])
        self.camera23_rot = np.array([-0.21783917758328675, 0.00017347121075593651, 0.97598435365204861])
        self.camera24_pos = np.array([9.983, 3.5])
        self.camera24_rot = np.array([-0.21783917758328675, 0.00017347121075593651, 0.97598435365204861])
        self.camera25_pos = np.array([9.983, 5.5])
        self.camera25_rot = np.array([-0.21783917758328675, 0.00017347121075593651, 0.97598435365204861])
        self.camera26_pos = np.array([-9.983, -5.75])
        self.camera26_rot = np.array([0.0, 0.21783924665317703, 0.0])
        self.camera27_pos = np.array([-9.983, -4.25])
        self.camera27_rot = np.array([0.0, 0.21783924665317703, 0.0])
        self.camera28_pos = np.array([-9.983, -2.75])
        self.camera28_rot = np.array([0.0, 0.21783924665317703, 0.0])
        self.camera29_pos = np.array([-9.983, -1.25])
        self.camera29_rot = np.array([0.0, 0.21783924665317703, 0.0])
        self.camera30_pos = np.array([-9.983, 0.25])
        self.camera30_rot = np.array([0.0, 0.21783924665317703, 0.0])
        self.camera31_pos = np.array([-9.983, 1.75])
        self.camera31_rot = np.array([0.0, 0.21783924665317703, 0.0])
        self.camera32_pos = np.array([-9.983, 3.25])
        self.camera32_rot = np.array([0.0, 0.21783924665317703, 0.0])
        self.camera33_pos = np.array([-9.983, 4.75])
        self.camera33_rot = np.array([0.0, 0.21783924665317703, 0.0])
        self.camera34_pos = np.array([-9.983, 6])
        self.camera34_rot = np.array([0.0, 0.21783924665317703, 0.0])
        self.camera_position_vec = 2
        self.image_position_vec = 30        
        self.xlimit = [-25, 25]
        self.ylimit = [-15, 15]       
        self.hfov_limit = 0.2182 
        self.dist = 0

        self.last_pose = [0.0, 0.0]
        self.tolerance = 0.01
        
        #subscriber dos angulos das cameras
        self.image_angle_subscription = self.create_subscription(
            ImagesAngles,
            '/image_angles',
            self.triangulation_callback,
            10
        )     
        self.robot_moving = False
        self.subscription_robo_movendo = self.create_subscription(
            Bool,
            '/robot_moving',
            self.robot_moving_callback,
            10
        )    

        # Desativar mensagens de retorno de chamada não utilizadas
        self.image_angle_subscription

        self.pose_publisher = self.create_publisher(PoseEstimate, 'pose_estimate', 10)
        self.rviz_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.pose_x = 0
        self.pose_y = 0
        
        

    def robot_moving_callback(self, msg):
        msg = Bool()
        self.robot_moving = msg.data

    def publish_pose_estimate(self):
        # Cria a mensagem de pose estimada e publica
        msg = PoseEstimate()
        msg.x = float(self.pose_x)  
        msg.y = float(self.pose_y) 
        self.pose_publisher.publish(msg)    

    def yaw_rotation(self,x,y,z):
        w = np.sqrt(1 - x**2 - y**2 - z**2)
        return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    
    def compara_ponto(self, camera_list, camera_desejada, ponto):
        a = round(ponto[0],2)
        b = round(ponto[1],2)
        if (camera_desejada == 0 or camera_desejada == 1 or camera_desejada == 2 or camera_desejada == 3 or camera_desejada == 4 or camera_desejada == 5
            or camera_desejada == 6 or camera_desejada == 7 or camera_desejada == 8 or camera_desejada == 9):
            return (b >= round(camera_list[0][1],2))
        elif (camera_desejada == 10 or camera_desejada == 11 or camera_desejada == 12 or camera_desejada == 13 or camera_desejada == 14 or camera_desejada == 15 
              or camera_desejada == 16 or camera_desejada == 17  or camera_desejada == 18):
            return (b <= round(camera_list[10][1],2))
        elif (camera_desejada == 19 or camera_desejada == 20 or camera_desejada == 21 or camera_desejada == 22 or camera_desejada == 23 or camera_desejada == 24 
              or camera_desejada == 25):
            return (a <= round(camera_list[19][0],2))
        elif (camera_desejada == 26 or camera_desejada == 27 or camera_desejada == 28 or camera_desejada == 29 or camera_desejada == 30 or camera_desejada == 31 
              or camera_desejada == 32 or camera_desejada == 33 or camera_desejada == 34):
            return (a >= round(camera_list[26][0],2))
    
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
        if not isnan(direcao[0]):
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
    
    def matplt_plotting_all(self, camera_position, camera_rotations, hfov_limit, image_angles_res, pontos_interseccao, bar_x, bar_y):
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

        #plotando os limites de hfov
        for pos, angle in zip(camera_position, hfov_limit):
            vector_length = 20 
            # Calcular os vetores de direção (usando o ângulo e a posição da câmera)
            vector_0 = np.array([np.cos(angle[0]), np.sin(angle[0])])  # Vetor limite inferior
            vector_1 = np.array([np.cos(angle[1]), np.sin(angle[1])])  # Vetor limite superior

            # Determinar os pontos finais dos vetores
            end_pos_0 = pos + self.camera_position_vec * vector_0
            end_pos_1 = pos + self.camera_position_vec * vector_1

            end_pos_area_0 = pos + vector_length * vector_0
            end_pos_area_1 = pos + vector_length * vector_1

            # Plotar os vetores
            ax.quiver(pos[0], pos[1], end_pos_0[0] - pos[0], end_pos_0[1] - pos[1], angles='xy', scale_units='xy', scale=1, color='k')
            ax.quiver(pos[0], pos[1], end_pos_1[0] - pos[0], end_pos_1[1] - pos[1], angles='xy', scale_units='xy', scale=1, color='k')

            # Calcular os vértices da área coberta e preenchê-la
            x_area = [pos[0], end_pos_area_0[0], end_pos_area_1[0]]
            y_area = [pos[1], end_pos_area_0[1], end_pos_area_1[1]]
            plt.fill(x_area, y_area, color='lightgray', alpha=0.5)

        for pos, angle in zip(camera_position, image_angles_res):
            if not isnan(angle):
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
    
    def plotting_all(self, camera_position, camera_rotations, hfov_limit, image_angles_res, pontos_interseccao, bar_x, bar_y):
        marker_array = MarkerArray()
        marker_id = 0
        # Plotar os vetores e posições das câmeras
        for pos, angle in zip(camera_position, camera_rotations):
            # Vetor da câmera
            vector_end = pos + np.array([self.camera_position_vec * np.cos(angle), self.camera_position_vec * np.sin(angle)])

            # Adicionar linha (vetor)
            line_marker = self.create_line_marker(marker_id, pos, vector_end, color=(1.0, 0.0, 0.0, 1.0))  # Vermelho
            marker_array.markers.append(line_marker)
            marker_id += 1

            # Adicionar posição (ponto)
            point_marker = self.create_point_marker(marker_id, pos, color=(0.0, 0.0, 1.0, 1.0))  # Azul
            marker_array.markers.append(point_marker)
            marker_id += 1       
        

        # Plotar os limites do HFOV
        for pos, angles in zip(camera_position, hfov_limit):
            for angle in angles:
                vector_end = pos + np.array([20 * np.cos(angle), 20 * np.sin(angle)])  # Vetores de limite

                line_marker = self.create_line_marker(marker_id, pos, vector_end, color=(0.0, 0.0, 0.0, 1.0))  # Preto
                marker_array.markers.append(line_marker)
                marker_id += 1

        # Área coberta pelo HFOV
        for pos, (angle_0, angle_1) in zip(camera_position, hfov_limit):
            point_0 = pos + np.array([20 * np.cos(angle_0), 20 * np.sin(angle_0)])
            point_1 = pos + np.array([20 * np.cos(angle_1), 20 * np.sin(angle_1)])

            area_marker = self.create_triangle_marker(marker_id, pos, point_0, point_1, color=(0.7, 0.7, 0.7, 0.15))  # Cinza claro
            marker_array.markers.append(area_marker)
            marker_id += 1

        # Plotar os pontos de interseção
        for point in pontos_interseccao:
            point_marker = self.create_point_marker(marker_id, point, color=(1.0, 1.0, 0.0, 1.0))  # Amarelo
            marker_array.markers.append(point_marker)
            marker_id += 1
  
        # Plotar o ponto baricentro
        bar_marker = self.create_point_marker(marker_id, [bar_x, bar_y], color=(0.0, 1.0, 1.0, 1.0))  # Ciano
        marker_array.markers.append(bar_marker)
        marker_id += 1

        # Publicar os markers no tópico
        self.rviz_publisher.publish(marker_array)

    def create_line_marker(self, marker_id, start, end, color):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "lines"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Espessura da linha

        # Cor
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        # Pontos da linha
        start_point = Point(x=start[0], y=start[1], z=0.0)
        end_point = Point(x=end[0], y=end[1], z=0.0)
        marker.points = [start_point, end_point]

        return marker
    
    def create_point_marker(self, marker_id, position, color):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "points"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Cor
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        # Posição
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.0

        return marker
    
    def create_triangle_marker(self, marker_id, point_0, point_1, point_2, color):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns = "hfov_areas"
        marker.id = marker_id
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD

        # Escala
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Cor
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color

        # Vértices do triângulo
        p0 = Point(x=point_0[0], y=point_0[1], z=0.0)
        p1 = Point(x=point_1[0], y=point_1[1], z=0.0)
        p2 = Point(x=point_2[0], y=point_2[1], z=0.0)
        marker.points = [p0, p1, p2]

        return marker
    
    def identificar_fileira(self, indice):
        """Identifica de qual fileira o índice pertence."""
        fileiras = {
            0: range(0, 10),      
            1: range(10, 19),     
            2: range(19, 26),     
            3: range(26, 35)      
        }     

        for fileira, indices in fileiras.items():
            if indice in indices:
                return fileira
        return None
        
    def estimate_pose(self, camera1_pos, camera2_pos, angle1, angle2):
        distance_vector = camera2_pos - camera1_pos
        d12 = np.linalg.norm(distance_vector)
        detection_vector1 = np.array([np.cos(angle1), np.sin(angle1)])
        detection_vector2 = np.array([np.cos(angle2), np.sin(angle2)])
        # Ângulo entre o vetor de detecção da câmera e o vetor distância
        angle_internal1 = np.arccos(
        np.dot(detection_vector1, distance_vector) / (np.linalg.norm(detection_vector1) * d12)
    )
        angle_internal2 = np.arccos(
        np.dot(detection_vector2, -distance_vector) / (np.linalg.norm(detection_vector2) * d12)
    )
        angle3 = np.pi - angle_internal1 - angle_internal2

        sin_angle1 = np.sin(angle_internal1)
        sin_angle2 = np.sin(angle_internal2)
        sin_angle3 = np.sin(angle3)

        A = (d12 * sin_angle2)/(sin_angle3)
        B = (d12 * sin_angle1)/(sin_angle3)

        est_1 = camera1_pos + A * detection_vector1
        est_2 = camera2_pos + B * detection_vector2

        result = (est_1 + est_2)/2

        return result  
    
            

    def triangulation_callback(self, msg):        
        camera_position = [self.camera0_pos, self.camera1_pos, self.camera2_pos, self.camera3_pos,
                           self.camera4_pos, self.camera5_pos, self.camera6_pos, self.camera7_pos,
                           self.camera8_pos, self.camera9_pos, self.camera10_pos, self.camera11_pos,
                           self.camera12_pos, self.camera13_pos, self.camera14_pos, self.camera15_pos,
                           self.camera16_pos, self.camera17_pos, self.camera18_pos, self.camera19_pos,
                           self.camera20_pos, self.camera21_pos, self.camera22_pos, self.camera23_pos, 
                           self.camera24_pos, self.camera25_pos, self.camera26_pos, self.camera27_pos, 
                           self.camera28_pos, self.camera29_pos, self.camera30_pos, self.camera31_pos, 
                           self.camera32_pos, self.camera33_pos, self.camera34_pos]
                           
        camera_rotations = [self.yaw_rotation(self.camera0_rot[0], self.camera0_rot[1], self.camera0_rot[2]), 
                     self.yaw_rotation(self.camera1_rot[0], self.camera1_rot[1], self.camera1_rot[2]),
                     self.yaw_rotation(self.camera2_rot[0], self.camera2_rot[1], self.camera2_rot[2]),
                     self.yaw_rotation(self.camera3_rot[0], self.camera3_rot[1], self.camera3_rot[2]),
                     self.yaw_rotation(self.camera4_rot[0], self.camera4_rot[1], self.camera4_rot[2]),
                     self.yaw_rotation(self.camera5_rot[0], self.camera5_rot[1], self.camera5_rot[2]),
                     self.yaw_rotation(self.camera6_rot[0], self.camera6_rot[1], self.camera6_rot[2]),
                     self.yaw_rotation(self.camera7_rot[0], self.camera7_rot[1], self.camera7_rot[2]),
                     self.yaw_rotation(self.camera8_rot[0], self.camera8_rot[1], self.camera8_rot[2]),                
                     self.yaw_rotation(self.camera9_rot[0], self.camera9_rot[1], self.camera9_rot[2]),
                     self.yaw_rotation(self.camera10_rot[0], self.camera10_rot[1], self.camera10_rot[2]),
                     self.yaw_rotation(self.camera11_rot[0], self.camera11_rot[1], self.camera11_rot[2]),
                     self.yaw_rotation(self.camera12_rot[0], self.camera12_rot[1], self.camera12_rot[2]),
                     self.yaw_rotation(self.camera13_rot[0], self.camera13_rot[1], self.camera13_rot[2]),
                     self.yaw_rotation(self.camera14_rot[0], self.camera14_rot[1], self.camera14_rot[2]),
                     self.yaw_rotation(self.camera15_rot[0], self.camera15_rot[1], self.camera15_rot[2]),
                     self.yaw_rotation(self.camera16_rot[0], self.camera16_rot[1], self.camera16_rot[2]),
                     self.yaw_rotation(self.camera17_rot[0], self.camera17_rot[1], self.camera17_rot[2]),
                     self.yaw_rotation(self.camera18_rot[0], self.camera18_rot[1], self.camera18_rot[2]),
                     self.yaw_rotation(self.camera19_rot[0], self.camera19_rot[1], self.camera19_rot[2]),
                     self.yaw_rotation(self.camera20_rot[0], self.camera20_rot[1], self.camera20_rot[2]),
                     self.yaw_rotation(self.camera21_rot[0], self.camera21_rot[1], self.camera21_rot[2]),
                     self.yaw_rotation(self.camera22_rot[0], self.camera22_rot[1], self.camera22_rot[2]),
                     self.yaw_rotation(self.camera23_rot[0], self.camera23_rot[1], self.camera23_rot[2]),
                     self.yaw_rotation(self.camera24_rot[0], self.camera24_rot[1], self.camera24_rot[2]),
                     self.yaw_rotation(self.camera25_rot[0], self.camera25_rot[1], self.camera25_rot[2]),
                     self.yaw_rotation(self.camera26_rot[0], self.camera26_rot[1], self.camera26_rot[2]),
                     self.yaw_rotation(self.camera27_rot[0], self.camera27_rot[1], self.camera27_rot[2]),
                     self.yaw_rotation(self.camera28_rot[0], self.camera28_rot[1], self.camera28_rot[2]),
                     self.yaw_rotation(self.camera29_rot[0], self.camera29_rot[1], self.camera29_rot[2]),
                     self.yaw_rotation(self.camera30_rot[0], self.camera30_rot[1], self.camera30_rot[2]),
                     self.yaw_rotation(self.camera31_rot[0], self.camera31_rot[1], self.camera31_rot[2]),
                     self.yaw_rotation(self.camera32_rot[0], self.camera32_rot[1], self.camera32_rot[2]),
                     self.yaw_rotation(self.camera33_rot[0], self.camera33_rot[1], self.camera33_rot[2]),
                     self.yaw_rotation(self.camera34_rot[0], self.camera34_rot[1], self.camera34_rot[2])
                     ]                
        #print('Angle image 0: ', msg.angle_image_0, 'Angle image 1: ', msg.angle_image_1, 'Angle image 2: ', msg.angle_image_2, 'Angle image 3: ', msg.angle_image_3)     
           
        up_hfov = 0
        down_hfov = 0
        hfov_limit = []
        for i in range(len(camera_rotations)):
            up_hfov = camera_rotations[i] + self.hfov_limit
            down_hfov = camera_rotations[i] - self.hfov_limit
            hfov_limit.append([up_hfov, down_hfov])
        #print('hfov limite 9: ', hfov_limit[9])      
        #print('rot: ', camera_rotations)
        
        image_angles = msg.angles 

        image_angles_res = []

        for i in range(len(image_angles)):
            if isnan(image_angles[i]):
                image_angles_res.append(float('nan'))
            elif (camera_rotations[i] + image_angles[i] > hfov_limit[i][0]) or (camera_rotations[i] + image_angles[i] < hfov_limit[i][1]):
                image_angles_res.append(float('nan'))
            else:
                image_angles_res.append(camera_rotations[i] - image_angles[i])

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
            if not isnan(image_angles_res[i]):
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
        pares_ortogonais = {(0, 3), (3, 0), (1, 3), (3, 1), (1, 2), (2, 1), (0, 2), (2, 0)}
        #print(len(retas))
        for i in range(len(image_angles_res)):
            for j in range(i + 1, len(image_angles_res)):
                fileira_i = self.identificar_fileira(i)
                fileira_j = self.identificar_fileira(j)

                # Verificar se a combinação de fileiras é permitida
                if (fileira_i, fileira_j) in pares_ortogonais:
                    #A1, B1, C1 = retas[i]
                    #A2, B2, C2 = retas[j]

                    # Calcular o ponto de interseção
                    #ponto = self.intersecao_retas(A1, B1, C1, A2, B2, C2, camera_position, i, j)
                    ponto = self.estimate_pose(camera_position[i], camera_position[j], image_angles_res[i], image_angles_res[j])


                    # Adicionar apenas pontos válidos dentro do mapa
                    if ponto is not None and self.mapa.verifica_ponto_dentro(ponto):
                        pontos_interseccao.append(ponto)

        #plotando
        
        bar_x, bar_y = self.baricentro(pontos_interseccao)
        self.pose_x = float(bar_x)
        self.pose_y = float(bar_y) 
        self.dist = sqrt((((self.pose_x - self.last_pose[0])**2 + (self.pose_y - self.last_pose[1])**2)))  
        if((self.pose_x != 0.0 and self.pose_y != 0.0) and not self.robot_moving and (self.dist >= self.tolerance)):
            self.publish_pose_estimate()
            self.last_pose = [self.pose_x, self.pose_y]    
        #self.plotting_all(camera_position, camera_rotations, hfov_limit, image_angles_res, pontos_interseccao, self.pose_x, self.pose_y)
        #self.matplt_plotting_all(camera_position, camera_rotations, hfov_limit, image_angles_res, pontos_interseccao, self.pose_x, self.pose_y)
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

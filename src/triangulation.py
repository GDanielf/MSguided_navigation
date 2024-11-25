#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from guided_navigation.msg import ImagesAngles
from guided_navigation.msg import PoseEstimate
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
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
        self.camera8_pos = np.array([6.0, -7.4687]) 
        self.camera8_rot = np.array([-0.15397426496748834, 0.15409692764667762, 0.68985053625575676])
        self.camera9_pos = np.array([6.0, 7.4687])
        self.camera9_rot = np.array([0.15397426496748834, 0.15409692764667762, -0.68985053625575676])
        self.camera10_pos = np.array([-6.0, -7.4687])
        self.camera10_rot = np.array([-0.15397426496748834, 0.15409692764667762, 0.68985053625575676])
        self.camera11_pos = np.array([-6.0, 7.4687])
        self.camera11_rot = np.array([0.15397426496748834, 0.15409692764667762, -0.68985053625575676])
        self.camera12_pos = np.array([9.983, -5])
        self.camera12_rot = np.array([-0.21783917758328675, 0.00017347121075593651, 0.97598435365204861])
        self.camera13_pos = np.array([9.983, 5])
        self.camera13_rot = np.array([-0.21783917758328675, 0.00017347121075593651, 0.97598435365204861])
        self.camera14_pos = np.array([-9.983, -5])
        self.camera14_rot = np.array([0.0, 0.21783924665317703, 0.0])
        self.camera15_pos = np.array([-9.983, 5])
        self.camera15_rot = np.array([0.0, 0.21783924665317703, 0.0])
        self.camera_position_vec = 2
        self.image_position_vec = 30        
        self.xlimit = [-25, 25]
        self.ylimit = [-15, 15]       
        self.hfov_limit = 0.1745 

        #subscriber dos angulos das cameras
        self.image_angle_subscription = self.create_subscription(
            ImagesAngles,
            '/image_angles',
            self.triangulation_callback,
            10
        )

        # Desativar mensagens de retorno de chamada não utilizadas
        self.image_angle_subscription

        self.publisher = self.create_publisher(PoseEstimate, 'pose_estimate', 10)
        self.rviz_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.pose_x = 0
        self.pose_y = 0

    def publish_pose_estimate(self):
        # Cria a mensagem de pose estimada e publica
        msg = PoseEstimate()
        msg.x = float(self.pose_x)  
        msg.y = float(self.pose_y) 
        self.publisher.publish(msg)    

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
            return (a >= round(camera_list[6][0],2))
        elif camera_desejada == 7:
            return (a >= round(camera_list[7][0],2))
        elif camera_desejada == 8:
            return (b >= round(camera_list[8][1],2))
        elif camera_desejada == 9:
            return (b <= round(camera_list[9][1],2))
        elif camera_desejada == 10:
            return (b >= round(camera_list[10][1],2))
        elif camera_desejada == 11:
            return (b <= round(camera_list[11][1],2))    
        elif camera_desejada == 12:
            return (a <= round(camera_list[12][0],2))    
        elif camera_desejada == 13:
            return (a <= round(camera_list[13][0],2)) 
        elif camera_desejada == 14:
            return (a >= round(camera_list[14][0],2))     
        elif camera_desejada == 15:
            return (a >= round(camera_list[15][0],2))   
    
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
        
    def triangulation_callback(self, msg):        
        camera_position = [self.camera0_pos, self.camera1_pos, self.camera2_pos, self.camera3_pos,
                           self.camera4_pos, self.camera5_pos, self.camera6_pos, self.camera7_pos,
                           self.camera8_pos, self.camera9_pos, self.camera10_pos, self.camera11_pos,
                           self.camera12_pos, self.camera13_pos, self.camera14_pos, self.camera15_pos]
                           
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
                     self.yaw_rotation(self.camera15_rot[0], self.camera15_rot[1], self.camera15_rot[2])
                     ]                
        #print('Angle image 0: ', msg.angle_image_0, 'Angle image 1: ', msg.angle_image_1, 'Angle image 2: ', msg.angle_image_2, 'Angle image 3: ', msg.angle_image_3)     
           
        up_hfov = 0
        down_hfov = 0
        hfov_limit = []
        for i in range(len(camera_rotations)):
            up_hfov = camera_rotations[i] + self.hfov_limit
            down_hfov = camera_rotations[i] - self.hfov_limit
            hfov_limit.append([up_hfov, down_hfov])
        print('hfov limite 9: ', hfov_limit[9])      
        #print('rot: ', camera_rotations)
        
        image_angles = [msg.angle_image_0, msg.angle_image_1, msg.angle_image_2, msg.angle_image_3,
                         msg.angle_image_4, msg.angle_image_5, msg.angle_image_6, msg.angle_image_7,
                         msg.angle_image_8, msg.angle_image_9, msg.angle_image_10, msg.angle_image_11,
                         msg.angle_image_12, msg.angle_image_13, msg.angle_image_14, msg.angle_image_15
                        ]

        image_angles_res = []

        for i in range(len(image_angles)):
            if math.isnan(image_angles[i]):
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
        self.publish_pose_estimate()
        self.plotting_all(camera_position, camera_rotations, hfov_limit, image_angles_res, pontos_interseccao, self.pose_x, self.pose_y)
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

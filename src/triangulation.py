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
from sensor_msgs.msg import JointState


class Triangulation(Node):
    def __init__(self):
        super().__init__('triangulation')
        self.mapa = Mapa()
        #Posicoes e rotacoes das cameras no mundo

        self.camera0_pos = np.array([-3, -7.4687]) #rot_z = 1.57
        self.camera0_rot = np.array([1.57, 0]) #rot z,y
        self.camera1_pos = np.array([4.0, 7.4687])
        self.camera1_rot = np.array([-1.57, 0])
        self.camera2_pos = np.array([-9.9830, -1.2500])
        self.camera2_rot = np.array([0, 0])        
        self.camera_position_vet = 2
        self.image_position_vet = 30        
        self.xlimit = [-25, 25]
        self.ylimit = [-15, 15]    

        self.last_pose = [0.0, 0.0]
        self.tolerance = 0.01

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.subscription  # Evita que o garbage collector remova a assinatura
        
        #subscriber dos angulos das cameras
        self.image_angle_subscription = self.create_subscription(
            ImagesAngles,
            '/image_angles',
            self.triangulation_callback,
            10
        )        

        # Desativar mensagens de retorno de chamada não utilizadas
        self.image_angle_subscription

        self.pose_publisher = self.create_publisher(PoseEstimate, 'pose_estimate', 10)
        self.rviz_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.pose_x = 0
        self.pose_y = 0

    def joint_state_callback(self, msg):
        joint_positions = {}
        for i, name in enumerate(msg.name):
            joint_positions[name] = msg.position[i]

        # Exibir as posições das juntas desejadas
        joints_of_interest = [
            "camera_tilt_joint_0",
            "head_yaw_joint_0",
            "camera_tilt_joint_1",
            "head_yaw_joint_1",
            "camera_tilt_joint_2",
            "head_yaw_joint_2"
        ]
        
        for joint in joints_of_interest:
            if joint in joint_positions:
                self.get_logger().info(f"{joint}: {joint_positions[joint]}")

    def publish_pose_estimate(self):
        # Cria a mensagem de pose estimada e publica
        msg = PoseEstimate()
        msg.x = float(self.pose_x)  
        msg.y = float(self.pose_y) 
        self.pose_publisher.publish(msg)    

    def yaw_rotation(self,x,y,z):
        w = np.sqrt(1 - x**2 - y**2 - z**2)
        return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))   
    
    def pontos_medio(self, lista_pontos_estimados):
        x = 0
        y = 0
        if(lista_pontos_estimados != []):
            for i in range(len(lista_pontos_estimados)):
                x = x + lista_pontos_estimados[i][0]
                y = y + lista_pontos_estimados[i][1]        
            x = x/(len(lista_pontos_estimados))
            y = y/(len(lista_pontos_estimados))
        return x,y  
    
    def matplt_plotting_all(self, camera_position, camera_rotations, image_angles_res, pontos_estimados, pontos_medio_x, pontos_medio_y):
        fig, ax = plt.subplots()
        # Plotar cada ponto e vetor da camera        
        for pos, angle in zip(camera_position, camera_rotations):
            # Calcular o vetor unitário
            unit_vector = np.array([self.camera_position_vet*np.cos(angle), self.camera_position_vet*np.sin(angle)])

            # Plotar a linha do vetor unitário
            ax.quiver(pos[0], pos[1], unit_vector[0], unit_vector[1], angles='xy', scale_units='xy', scale=1, color= 'r')

            # Plotar a posição
            ax.scatter(pos[0], pos[1], color='b')        

        #plotando os limites de hfov
        for pos, angle in zip(camera_position):
            vector_length = 20 
            # Calcular os vetores de direção (usando o ângulo e a posição da câmera)
            vector_0 = np.array([np.cos(angle[0]), np.sin(angle[0])])  # Vetor limite inferior
            vector_1 = np.array([np.cos(angle[1]), np.sin(angle[1])])  # Vetor limite superior

            # Determinar os pontos finais dos vetores
            end_pos_0 = pos + self.camera_position_vet * vector_0
            end_pos_1 = pos + self.camera_position_vet * vector_1

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
                final_vector = np.array([self.image_position_vet*np.cos(angle), self.image_position_vet*np.sin(angle)])

                # Plotar a linha do vetor unitário
                ax.quiver(pos[0], pos[1], final_vector[0], final_vector[1], angles='xy', scale_units='xy', scale=1, color= 'g')

                # Plotar a posição
                ax.scatter(pos[0], pos[1], color='b')   


        # Plotar os pontos de interseccao
        for j in pontos_estimados:       
            # Plotar a posição
            ax.scatter(j[0], j[1], color='y')  


        ax.scatter(float(pontos_medio_x), float(pontos_medio_y), color='c')        
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
    
    def plotting_all(self, camera_position, camera_rotations, image_angles_res, pontos_estimados, pontos_medio_x, pontos_medio_y):
        marker_array = MarkerArray()
        marker_id = 0
        # Plotar os vetores e posições das câmeras
        for pos, angle in zip(camera_position, camera_rotations):
            # Vetor da câmera
            vector_end = pos + np.array([self.camera_position_vet * np.cos(angle), self.camera_position_vet * np.sin(angle)])

            # Adicionar linha (vetor)
            line_marker = self.create_line_marker(marker_id, pos, vector_end, color=(1.0, 0.0, 0.0, 1.0))  # Vermelho
            marker_array.markers.append(line_marker)
            marker_id += 1

            # Adicionar posição (ponto)
            point_marker = self.create_point_marker(marker_id, pos, color=(0.0, 0.0, 1.0, 1.0))  # Azul
            marker_array.markers.append(point_marker)
            marker_id += 1       
        

        # Plotar os limites do HFOV
        for pos, angles in zip(camera_position):
            for angle in angles:
                vector_end = pos + np.array([20 * np.cos(angle), 20 * np.sin(angle)])  # Vetores de limite

                line_marker = self.create_line_marker(marker_id, pos, vector_end, color=(0.0, 0.0, 0.0, 1.0))  # Preto
                marker_array.markers.append(line_marker)
                marker_id += 1

        # Área coberta pelo HFOV
        for pos, (angle_0, angle_1) in zip(camera_position):
            point_0 = pos + np.array([20 * np.cos(angle_0), 20 * np.sin(angle_0)])
            point_1 = pos + np.array([20 * np.cos(angle_1), 20 * np.sin(angle_1)])

            area_marker = self.create_triangle_marker(marker_id, pos, point_0, point_1, color=(0.7, 0.7, 0.7, 0.15))  # Cinza claro
            marker_array.markers.append(area_marker)
            marker_id += 1

        # Plotar os pontos de interseção
        for point in pontos_estimados:
            point_marker = self.create_point_marker(marker_id, point, color=(1.0, 1.0, 0.0, 1.0))  # Amarelo
            marker_array.markers.append(point_marker)
            marker_id += 1
  
        # Plotar o ponto pontos_medio
        bar_marker = self.create_point_marker(marker_id, [pontos_medio_x, pontos_medio_y], color=(0.0, 1.0, 1.0, 1.0))  # Ciano
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
        camera_position = [self.camera0_pos, self.camera1_pos, self.camera2_pos]
                           
        camera_rotations = [self.yaw_rotation(self.camera0_rot[0], self.camera0_rot[1], self.camera0_rot[2]), 
                     self.yaw_rotation(self.camera1_rot[0], self.camera1_rot[1], self.camera1_rot[2]),
                     self.yaw_rotation(self.camera2_rot[0], self.camera2_rot[1], self.camera2_rot[2])
                     ]                
        #print('Angle image 0: ', msg.angle_image_0, 'Angle image 1: ', msg.angle_image_1, 'Angle image 2: ', msg.angle_image_2, 'Angle image 3: ', msg.angle_image_3)     
           
          
        #print('rot: ', camera_rotations)
        
        image_angles = msg.angles 

        image_angles_res = []

        for i in range(len(image_angles)):
            if isnan(image_angles[i]):
                image_angles_res.append(float('nan'))
            else:
                image_angles_res.append(camera_rotations[i] - image_angles[i])
        
        #print('retas: ', retas)
        #print('ang: ', image_angles)
        #print('ang tratado: ', image_angles_res)
        #print('pos tratado: ', camera_rotations)  

        pontos_estimados = []
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
                        pontos_estimados.append(ponto)

        #plotando
        
        pontos_medio_x, pontos_medio_y = self.pontos_medio(pontos_estimados)
        self.pose_x = float(pontos_medio_x)
        self.pose_y = float(pontos_medio_y)         
        if((self.pose_x != 0.0 and self.pose_y != 0.0)):
            self.publish_pose_estimate()
            #self.last_pose = [self.pose_x, self.pose_y]    
        self.plotting_all(camera_position, camera_rotations, image_angles_res, pontos_estimados, self.pose_x, self.pose_y)
        #self.matplt_plotting_all(camera_position, camera_rotations, image_angles_res, pontos_estimados, self.pose_x, self.pose_y)
        #print('pontos de intersec: ', pontos_estimados)
        #print('pontos_medio: ', [pontos_medio_x, pontos_medio_y])
        #pontos_insterseccao = pos_list()
        #print(f"Ponto de interseção : {pontos_estimados}")
    
        
def main(args=None):
    rclpy.init(args=args)
    node = Triangulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()

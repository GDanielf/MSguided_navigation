#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64MultiArray
from guided_navigation.srv import SetCameraActive
from cv_bridge import CvBridge
import cv2
import inference
import numpy as np
import math
from message_filters import Subscriber, TimeSynchronizer
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from guided_navigation.msg import PoseEstimate
from scipy.spatial.transform import Rotation

#from guided_navigation.msg import Rectangle
class MultiCamera(Node):
    def __init__(self):
        super().__init__('multi_camera')
        self.bridge = CvBridge()        

        #configuracao da caixa delimitador de deteccao
        self.font_size = 1.5
        self.font_color = (0, 0, 255)
        self.font_thickness = 2
        self.image_width = 640
        self.image_height = 480

        #dividir a imagem ao meio
        self.thickness = 0.5
        self.mid_x = self.image_width // 2
        self.mid_y = self.image_height // 2
        self.start_point_vertical = (self.mid_x, 0) 
        self.end_point_vertical = (self.mid_x, self.image_height)
        self.start_point_horiozontal = (0, self.mid_y) 
        self.end_point_horiozontal = (self.image_width, self.mid_y)
        self.line_color = (0, 0, 0)
        self.thickness = 2  
        #parametros de triangulacao
        self.camera0_pos = np.array([-3, -7.4687]) #rot_z = 1.57
        self.camera0_rot = np.array([0, 0, np.pi/2]) #rot z,y
        self.camera1_pos = np.array([4.0, 7.4687])
        self.camera1_rot = np.array([0, 0, -np.pi/2])
        self.camera2_pos = np.array([-9.9830, -1.2500])
        self.camera2_rot = np.array([0, 0, 0])  
        self.pose_x = 0
        self.pose_y = 0

        # Configuração do Modelo
        self.model = inference.get_model("husky_test/3")

        self.subscription_robo_movendo = self.create_subscription(
            Bool, '/robot_moving', self.robot_moving_callback, 10
        )   
        self.simulation_subscriber = self.create_subscription(
            Bool, '/simulation_status', self.simulation_callback, 10
        ) 
        self.create_subscription(
            Clock, '/clock', self.clock_callback, 10  
        )

        self.pose_publisher = self.create_publisher(PoseEstimate, 'pose_estimate', 10)                  

        #joints
        self.velocity_publishers = {
            "camera_joint_0": self.create_publisher(Float64MultiArray, 'velocity_controller_0/commands', 10),
            "camera_joint_1": self.create_publisher(Float64MultiArray, 'velocity_controller_1/commands', 10),
            "camera_joint_2": self.create_publisher(Float64MultiArray, 'velocity_controller_2/commands', 10),
        }
        self.tilt_joints = [
            "camera_tilt_joint_0",
            "camera_tilt_joint_1",
            "camera_tilt_joint_2",
        ]
        self.yaw_joints = [
            "head_yaw_joint_0",
            "head_yaw_joint_1",
            "head_yaw_joint_2"
        ]
        self.initial_target_position = [0.5, 0.5, 0.4]
        self.kp = 1
        self.kp_cam_yaw = 0.00025
        self.kp_cam_tilt = 0.00025
        self.threshold = 0.01
        self.threshold_pixel = 2
        self.iniciar_posicao = [False, False, False]
        self.centralizar_posicao = [False, False, False]
        self.joint_imagem_centralizada = [[0.0, 0.0],[0.0, 0.0],[0.0, 0.0]] #index 0 = camera 0 [yaw,tilt]
        self.joint_positions = {}
        
        self.joints_to_control = ["camera_joint_0", "camera_joint_1", "camera_joint_2"]
        self.x_centro = 1920 // 2
        self.y_centro = 1080 // 2

        self.active_cameras = {i: False for i in range(3)}     

        self.robot_moving = False 
        self.simulation_active = False         
        self.simulation_time = None 
        self.robot_stopped_time = None

        #estabilidade do ponto estimado
        self.contador_ponto_estavel = 0
        self.N = 10
        self.valores_x = np.zeros(self.N)
        self.valores_y = np.zeros(self.N)

        self.subscription_joint = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.subscription_joint  # Evita que o garbage collector remova a assinatura

        # Subscrições para os tópicos de imagem        
        self.camera_subscribers = []
        for i in range(3):
            topic_name = f'/world/empty/model/camera_robot/link/head_link_{i}/sensor/camera_sensor_{i}/image'
            subscriber = Subscriber(self, Image, topic_name)
            self.camera_subscribers.append(subscriber)
            
        #sincronizacao das imagens
        self.sync = TimeSynchronizer(self.camera_subscribers, 10)
        self.sync.registerCallback(self.camera_callback)
        
        # Criar o servidor de serviço para ativar/desativar câmeras
        self.srv = self.create_service(SetCameraActive, 'set_camera_active', self.set_camera_active)
        
    def clock_callback(self, msg):
        self.simulation_time = msg.clock

    def publish_pose_estimate(self):
        msg = PoseEstimate()
        msg.x = float(self.pose_x)  
        msg.y = float(self.pose_y) 
        self.pose_publisher.publish(msg) 

    def simulation_callback(self, msg):
        self.simulation_active = msg.data

    def robot_moving_callback(self, msg):
        self.robot_moving = msg.data         

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]

        for index, joint_name in enumerate(self.tilt_joints):
            if not self.iniciar_posicao[index]:
                self.inicializar_posicao_y_joint(joint_name, index)
                break
        if(all(self.iniciar_posicao) and not self.robot_moving and all(self.centralizar_posicao)): # esse recebe o valor do planner
            pontos_estimados = []
            camera_position = [self.camera0_pos, self.camera1_pos, self.camera2_pos]                           
             
            rot_0 = Rotation.from_euler('xyz', [self.camera0_rot[0], self.camera0_rot[1] + self.joint_imagem_centralizada[0][1], self.camera0_rot[2] + self.joint_imagem_centralizada[0][0]])
            rot_1 = Rotation.from_euler('xyz', [self.camera1_rot[0], self.camera1_rot[1] + self.joint_imagem_centralizada[1][1], self.camera1_rot[2] + self.joint_imagem_centralizada[1][0]])
            rot_2 = Rotation.from_euler('xyz', [self.camera2_rot[0], self.camera2_rot[1] + self.joint_imagem_centralizada[2][1], self.camera2_rot[2] + self.joint_imagem_centralizada[2][0]])           
                
            camera_rotations = [rot_0.as_euler('xyz')[2], rot_1.as_euler('xyz')[2], rot_2.as_euler('xyz')[2]]
            
            print(camera_rotations)
            for i in range(3):
                for j in range(i + 1, 3):
                    pontos_estimados.append(self.estimate_pose(camera_position[i], camera_position[j], camera_rotations[i], camera_rotations[j])) 

            pontos_medio_x, pontos_medio_y = self.pontos_medio(pontos_estimados)
            self.pose_x = float(pontos_medio_x)
            self.pose_y = float(pontos_medio_y)
            if(self.ponto_estavel(self.pose_x, self.pose_y, 0.01)):
                self.publish_pose_estimate()
                self.contador_ponto_estavel = 0 
                self.valores_x = np.zeros(self.N)
                self.valores_y = np.zeros(self.N)      

    def inicializar_posicao_y_joint(self, joint_name, index):
        velocity_msg = Float64MultiArray()
        current_pos = self.joint_positions[joint_name]
        error = current_pos - self.initial_target_position[index]
        if abs(error) < self.threshold:
            self.iniciar_posicao[index] = True
            velocity_msg.data = [0.0, 0.0]  
            self.velocity_publishers[self.joints_to_control[index]].publish(velocity_msg)
            self.get_logger().info(f"{joint_name} ajustada! [{self.iniciar_posicao}]")
            return 
        velocity = -self.kp * error
        velocity_msg.data = [0.0, velocity]
        self.velocity_publishers[self.joints_to_control[index]].publish(velocity_msg)

        self.get_logger().info(f"Ajustando {joint_name}: pos {current_pos:.3f} → vel {velocity:.3f}")

    def ponto_estavel(self, novo_x, novo_y, tolerancia):
        self.valores_x[:-1] = self.valores_x[1:]
        self.valores_x[-1] = novo_x
        self.valores_y[:-1] = self.valores_y[1:]
        self.valores_y[-1] = novo_y
        if(self.contador_ponto_estavel < self.N):
            self.contador_ponto_estavel += 1
            return False
        return  (np.ptp(self.valores_x) < tolerancia) and (np.ptp(self.valores_y) < tolerancia)
 
    def camera_callback(self, *camera_images):
        velocity_msg = Float64MultiArray()
        if all(self.iniciar_posicao):
            for camera_id, msg in enumerate(camera_images):
                # Converte a imagem ROS para OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                
                # Realiza a inferência
                predict = self.model.infer(image=cv_image)
                
                if not predict[0].predictions:
                    self.get_logger().info(f"Camera {camera_id} nao identificou o robo")
                else:
                    for prediction in predict[0].predictions:
                        x, y = int(prediction.x), int(prediction.y)
                        width, height = int(prediction.width), int(prediction.height)
                        top_left = (abs(int(width / 2) - x), abs(int(height / 2) - y))
                        bottom_right = (int(width / 2) + x, int(height / 2) + y)
                        cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)
                        label = f"{prediction.class_name}: {prediction.confidence:.5f}"
                        cv2.putText(cv_image, label, top_left, 
                                cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_color, self.font_thickness, cv2.LINE_AA)                        
                        
                        erro_x = int(x) - int(self.x_centro)
                        erro_y = int(y) - int(self.y_centro)
                        if abs(erro_x) < self.threshold_pixel and abs(erro_y) < self.threshold_pixel:
                            velocity_msg.data = [0.0, 0.0]
                            self.velocity_publishers[self.joints_to_control[camera_id]].publish(velocity_msg)
                            self.joint_imagem_centralizada[camera_id] =  [self.joint_positions[self.yaw_joints[camera_id]], 
                                                                    self.joint_positions[self.tilt_joints[camera_id]]]
                            self.centralizar_posicao[camera_id] = True
                        else:                       
                            #self.joint_imagem_centralizada[camera_id] = [float('nan'), float('nan')]
                            velocity_msg.data = [float(-self.kp_cam_yaw * erro_x), float(self.kp_cam_tilt * erro_y)]
                            self.velocity_publishers[self.joints_to_control[camera_id]].publish(velocity_msg)
                
                # Apenas exibir se a câmera for ativada
                if self.is_camera_active(camera_id):
                    self.visualize_camera(cv_image, camera_id)

    #triangulacao
    
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
                
            
    #servico para ativar as cameras com opencv
    def set_camera_active(self, request, response):
        camera_id = request.camera_id
        activate = request.activate

        # Ativar ou desativar a câmera
        if camera_id in self.active_cameras:
            self.active_cameras[camera_id] = activate
            self.get_logger().info(f"Câmera {camera_id} {'ativada' if activate else 'desativada'}.")
            response.success = True
            response.message = f"Câmera {camera_id} {'ativada' if activate else 'desativada'}."
        else:
            response.success = False
            response.message = f"Câmera {camera_id} não existe."
        
        return response

    def is_camera_active(self, camera_id):
        # Checa se a câmera está ativa para exibição
        return self.active_cameras.get(camera_id, False)

    def visualize_camera(self, image, camera_id):
        resized_image = cv2.resize(image, (self.image_width, self.image_height))
        cv2.line(resized_image, self.start_point_vertical, self.end_point_vertical, self.line_color, self.thickness)
        cv2.line(resized_image, self.start_point_horiozontal, self.end_point_horiozontal, self.line_color, self.thickness)
        cv2.imshow(f'Camera {camera_id}', resized_image)
        cv2.waitKey(1)

    def activate_camera(self, camera_id):
        if camera_id in self.active_cameras:
            self.active_cameras[camera_id] = True
            print(f"Câmera {camera_id} ativada.")
        else:
            print(f"Câmera {camera_id} não existe.")

    def deactivate_camera(self, camera_id):
        if camera_id in self.active_cameras:
            self.active_cameras[camera_id] = False
            print(f"Câmera {camera_id} desativada.")
        else:
            print(f"Câmera {camera_id} não existe.")


def main(args=None):
    rclpy.init(args=args)
    node = MultiCamera()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()

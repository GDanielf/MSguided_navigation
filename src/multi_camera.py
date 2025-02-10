#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64MultiArray
from guided_navigation.msg import ImagesAngles
from guided_navigation.srv import SetCameraActive
from cv_bridge import CvBridge
import cv2
import inference
import numpy as np
import math
from message_filters import Subscriber, TimeSynchronizer
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


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

        #angle_publisher para enviar os valores dos angulos timer_publition publica msg a cada 1 seg
        self.angle_publisher = self.create_publisher(ImagesAngles, 'image_angles', 10)

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
        self.threshold = 0.01
        self.iniciar_posicao = [False, False, False]
        self.joint_positions = {}
        self.joints_to_control = ["camera_joint_0", "camera_joint_1", "camera_joint_2"]

        self.angles = [float('nan')] * 3 
        self.active_cameras = {i: False for i in range(3)}     

        self.robot_moving = False 
        self.simulation_active = False         
        self.simulation_time = None 
        self.robot_stopped_time = None

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
        # Recebe o tempo de simulação
        self.simulation_time = msg.clock

    def simulation_callback(self, msg):
        self.simulation_active = msg.data

    def robot_moving_callback(self, msg):
        self.robot_moving = msg.data
        if not msg.data:  
            self.robot_stopped_time = self.simulation_time

    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]

        for index, joint_name in enumerate(self.tilt_joints):
            if not self.iniciar_posicao[index]:
                self.inicializar_posicao_y_joint(joint_name, index)
                break

    def inicializar_posicao_y_joint(self, joint_name, index):
        velocity_msg = Float64MultiArray()
        current_pos = self.joint_positions[joint_name]
        error = current_pos - self.initial_target_position[index]
        if abs(error) < self.threshold:
            self.iniciar_posicao[index] = True
            velocity_msg = Float64MultiArray()
            velocity_msg.data = [0.0, 0.0]  
            self.velocity_publishers[self.joints_to_control[index]].publish(velocity_msg)
            self.get_logger().info(f"{joint_name} ajustada! [{self.iniciar_posicao}]")
            return 
        velocity = -self.kp * error
        velocity_msg = Float64MultiArray()
        velocity_msg.data = [0.0, velocity]
        self.velocity_publishers[self.joints_to_control[index]].publish(velocity_msg)

        self.get_logger().info(f"Ajustando {joint_name}: pos {current_pos:.3f} → vel {velocity:.3f}")


    def publish_camera_angles(self):
        msg = ImagesAngles()
        msg.angles = self.angles
        self.angle_publisher.publish(msg)
    
    #funcao que recebe o ponto do centro do retangulo e retorna o angulo entre a reta que divide metade da imagem o ponto 

    def angulo_centro(self,x,y):        
        return math.atan((x - 959)/(1080 - y))

    def camera_callback(self, *camera_images):
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
                        self.angles[camera_id] = self.angulo_centro(x, y)
                        width, height = int(prediction.width), int(prediction.height)
                        top_left = (abs(int(width / 2) - x), abs(int(height / 2) - y))
                        bottom_right = (int(width / 2) + x, int(height / 2) + y)
                        cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)
                        label = f"{prediction.class_name}: {prediction.confidence:.5f}"
                        cv2.putText(cv_image, label, top_left, 
                                cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_color, self.font_thickness, cv2.LINE_AA)
                
                # Apenas exibir se a câmera for ativada
                if self.is_camera_active(camera_id):
                    self.visualize_camera(cv_image, camera_id)
                
            if(self.simulation_active and not self.robot_moving): 
                self.publish_camera_angles()    
        
            

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

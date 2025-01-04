#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from guided_navigation.msg import ImagesAngles
from guided_navigation.srv import SetCameraActive
from cv_bridge import CvBridge
import cv2
import inference
import numpy as np
import math
from message_filters import Subscriber, TimeSynchronizer
from rosgraph_msgs.msg import Clock

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
        self.start_point = (self.mid_x, 0)  # Ponto de início (meio da imagem, topo)
        self.end_point = (self.mid_x, self.image_height)
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

        self.angles = [float('nan')] * 35 
        self.active_cameras = {i: False for i in range(35)}     

        self.robot_moving = False 
        self.simulation_active = False         
        self.simulation_time = None 
        self.robot_stopped_time = None

        # Subscrições para os tópicos de imagem        
        self.camera_subscribers = []
        for i in range(35):
            topic_name = f'/world/empty/model/rgbd_camera_{i}/link/link_{i}/sensor/camera_sensor_{i}/image'
            subscriber = Subscriber(self, Image, topic_name)
            self.camera_subscribers.append(subscriber)
        
        queue_size = 10
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

    def publish_camera_angles(self):
        msg = ImagesAngles()
        msg.angles = self.angles
        self.angle_publisher.publish(msg)
    
    #funcao que recebe o ponto do centro do retangulo e retorna o angulo entre a reta que divide metade da imagem o ponto 

    def angulo_centro(self,x,y):
        angle = 0
        tang = (x - 959)/(1080 - y)
        angle = math.atan(tang)
        return angle

    def camera_callback(self, *camera_images):   
        for camera_id, msg in enumerate(camera_images):
            # Converte a imagem ROS para OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Realiza a inferência
            predict = self.model.infer(image=cv_image)
            
            if not predict[0].predictions:
                self.angles[camera_id] = float('nan')
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
        cv2.line(resized_image, self.start_point, self.end_point, self.line_color, self.thickness)
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

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from guided_navigation.msg import ImagesAngles
from guided_navigation.srv import SetCameraActive
from cv_bridge import CvBridge
import cv2
import inference
import numpy as np
import math

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

        #publisher para enviar os valores dos angulos timer_publition publica msg a cada 1 seg
        self.publisher = self.create_publisher(ImagesAngles, 'image_angles', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.angles = [float('nan')] * 35 
        self.active_cameras = {i: False for i in range(35)}

        # Subscrições para os tópicos de imagem
        
        # Aqui criamos a assinatura para cada câmera
        self.subscription0 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_0/link/link_0/sensor/camera_sensor_0/image',
            lambda msg: self.camera_callback(msg, 0),  
            10
        )

        self.subscription1 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_1/link/link1/sensor/camera_sensor_1/image',
            lambda msg: self.camera_callback(msg, 1),  
            10
        )

        self.subscription2 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_2/link/link_2/sensor/camera_sensor_2/image',
            lambda msg: self.camera_callback(msg, 2),  
            10
        )

        self.subscription3 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_3/link/link_3/sensor/camera_sensor_3/image',
            lambda msg: self.camera_callback(msg, 3),  
            10
        )

        self.subscription4 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_4/link/link_4/sensor/camera_sensor_4/image',
            lambda msg: self.camera_callback(msg, 4),  
            10
        )

        self.subscription5 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_5/link/link_5/sensor/camera_sensor_5/image',
            lambda msg: self.camera_callback(msg, 5),  
            10
        )

        self.subscription6 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_6/link/link_6/sensor/camera_sensor_6/image',
            lambda msg: self.camera_callback(msg, 6),  
            10
        )

        self.subscription7 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_7/link/link_7/sensor/camera_sensor_7/image',
            lambda msg: self.camera_callback(msg, 7),  
            10
        )

        self.subscription8 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_8/link/link_8/sensor/camera_sensor_8/image',
            lambda msg: self.camera_callback(msg, 8),  
            10
        )

        self.subscription9 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_9/link/link_9/sensor/camera_sensor_9/image',
            lambda msg: self.camera_callback(msg, 9),  
            10
        )

        self.subscription10 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_10/link/link_10/sensor/camera_sensor_10/image',
            lambda msg: self.camera_callback(msg, 10),  
            10
        )

        self.subscription11 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_11/link/link_11/sensor/camera_sensor_11/image',
            lambda msg: self.camera_callback(msg, 11),  
            10
        )

        self.subscription12 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_12/link/link_12/sensor/camera_sensor_12/image',
            lambda msg: self.camera_callback(msg, 12),  
            10
        )

        self.subscription13 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_13/link/link_13/sensor/camera_sensor_13/image',
            lambda msg: self.camera_callback(msg, 13),  
            10
        )

        self.subscription14 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_14/link/link_14/sensor/camera_sensor_14/image',
            lambda msg: self.camera_callback(msg, 14),  
            10
        )

        self.subscription15 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_15/link/link_15/sensor/camera_sensor_15/image',
            lambda msg: self.camera_callback(msg, 15),  
            10
        )

        self.subscription16 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_16/link/link_16/sensor/camera_sensor_16/image',
            lambda msg: self.camera_callback(msg, 16),  
            10
        )

        self.subscription17 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_17/link/link_17/sensor/camera_sensor_17/image',
            lambda msg: self.camera_callback(msg, 17),  
            10
        )

        self.subscription18 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_18/link/link_18/sensor/camera_sensor_18/image',
            lambda msg: self.camera_callback(msg, 18),  
            10
        )

        self.subscription19 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_19/link/link_19/sensor/camera_sensor_19/image',
            lambda msg: self.camera_callback(msg, 19),  
            10
        )

        self.subscription20 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_20/link/link_20/sensor/camera_sensor_20/image',
            lambda msg: self.camera_callback(msg, 20),  
            10
        )

        self.subscription21 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_21/link/link_21/sensor/camera_sensor_21/image',
            lambda msg: self.camera_callback(msg, 21),  
            10
        )

        self.subscription22 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_22/link/link_22/sensor/camera_sensor_22/image',
            lambda msg: self.camera_callback(msg, 22),  
            10
        )
        
        self.subscription23 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_23/link/link_23/sensor/camera_sensor_23/image',
            lambda msg: self.camera_callback(msg, 23),  
            10
        )

        self.subscription24 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_24/link/link_24/sensor/camera_sensor_24/image',
            lambda msg: self.camera_callback(msg, 24),  
            10
        )

        self.subscription25 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_25/link/link_25/sensor/camera_sensor_25/image',
            lambda msg: self.camera_callback(msg, 25),  
            10
        )

        self.subscription26 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_26/link/link_26/sensor/camera_sensor_26/image',
            lambda msg: self.camera_callback(msg, 26),  
            10
        )

        self.subscription27 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_27/link/link_27/sensor/camera_sensor_27/image',
            lambda msg: self.camera_callback(msg, 27),  
            10
        )

        self.subscription28 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_28/link/link_28/sensor/camera_sensor_28/image',
            lambda msg: self.camera_callback(msg, 28),  
            10
        )

        self.subscription29 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_29/link/link_29/sensor/camera_sensor_29/image',
            lambda msg: self.camera_callback(msg, 29),  
            10
        )

        self.subscription30 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_30/link/link_30/sensor/camera_sensor_30/image',
            lambda msg: self.camera_callback(msg, 30),  
            10
        )

        self.subscription31 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_31/link/link_31/sensor/camera_sensor_31/image',
            lambda msg: self.camera_callback(msg, 31),  
            10
        )

        self.subscription32 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_32/link/link_32/sensor/camera_sensor_32/image',
            lambda msg: self.camera_callback(msg, 32),  
            10
        )

        self.subscription33 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_33/link/link_33/sensor/camera_sensor_33/image',
            lambda msg: self.camera_callback(msg, 33),  
            10
        )

        self.subscription34 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_34/link/link_34/sensor/camera_sensor_34/image',
            lambda msg: self.camera_callback(msg, 34),  
            10
        ) 
        
        # Criar o servidor de serviço para ativar/desativar câmeras
        self.srv = self.create_service(SetCameraActive, 'set_camera_active', self.set_camera_active)
        

    def timer_callback(self):
        msg = ImagesAngles()
        msg.angles = self.angles
        self.publisher.publish(msg)
    
    #funcao que recebe o ponto do centro do retangulo e retorna o angulo entre a reta que divide metade da imagem o ponto 

    def angulo_centro(self,x,y):
        angle = 0
        tang = (x - 959)/(1080 - y)
        angle = math.atan(tang)
        return angle

    def camera_callback(self, msg, camera_id):
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
        
        # Apenas exibir se a câmera for ativada
        if self.is_camera_active(camera_id):
            self.visualize_camera(cv_image, camera_id)

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

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from guided_navigation.msg import ImagesAngles
from cv_bridge import CvBridge
import cv2
import inference
import numpy as np
import math

#from guided_navigation.msg import Rectangle
class MultiCamera(Node):
    def __init__(self):
        super().__init__('multi_camera')

        self.image = None
        self.bridge = CvBridge()        

        #configuracao da caixa delimitador de deteccao
        self.font_size = 1.5
        self.font_color = (0, 0, 255)
        self.font_thickness = 2

        # Configuração do Modelo
        self.model = inference.get_model("husky_test/3")

        #publisher para enviar os valores dos angulos timer_publition publica msg a cada 1 seg
        self.publisher = self.create_publisher(ImagesAngles, 'image_angles', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Subscrições para os 4 tópicos de imagem
        self.subscription1 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera/link/link/sensor/camera_sensor/image',
            self.camera1_callback,
            10)
        
        self.subscription2 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_1/link/link_1/sensor/camera_sensor_1/image',
            self.camera2_callback,
            10)
        
        self.subscription3 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_2/link/link_2/sensor/camera_sensor_2/image',
            self.camera3_callback,
            10)
        
        self.subscription4 = self.create_subscription(
            Image,
            '/world/empty/model/rgbd_camera_3/link/link_3/sensor/camera_sensor_3/image',
            self.camera4_callback,
            10)       

        self.image = None
        # Desativar mensagens de retorno de chamada não utilizadas
        self.subscription1
        self.subscription2
        self.subscription3
        self.subscription4
        
        # angulos de cada camera para enviar (float)
        self.angle1 = 0
        self.angle2 = 0
        self.angle3 = 0
        self.angle4 = 0        

    def timer_callback(self):
        msg = ImagesAngles()
        msg.angle_image_1 = float(self.angle1)
        msg.angle_image_2 = float(self.angle2)
        msg.angle_image_3 = float(self.angle3)
        msg.angle_image_4 = float(self.angle4)
        self.publisher.publish(msg)
        #self.get_logger().info(f'Published angles: {msg.angle_image_1}, {msg.angle_image_2}, {msg.angle_image_3}, {msg.angle_image_4}')

    
    #funcao que recebe o ponto do centro do retangulo e retorna o angulo entre a reta que divide metade da imagem o ponto 

    def angulo_centro(self,x,y):
        angle = 0
        tang = (x - 959)/y
        angle = math.atan(tang)

        # Converter o ângulo para graus
        #angle_degrees = math.degrees(angle)

        return angle

    def camera1_callback(self, msg):
        #self.get_logger().info('Recebida imagem da câmera 1')
        # Processamento da imagem da câmera 1
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        predict = self.model.infer(image = cv_image)
        #print(predict)
        for prediction in predict[0].predictions:
            # Extrai as coordenadas e dimensões da caixa delimitadora
            x = int(prediction.x)
            y = int(prediction.y)
            width = int(prediction.width)
            height = int(prediction.height)
            self.angle1 = self.angulo_centro(x,y)
            #print('Camera 1 - x: ', x, 'y: ', y, 'angle: ', self.angle1)

            # Calcula as coordenadas dos cantos da caixa
            top_left = (abs(int(width/2) - x), abs(int(height/2) - y))
            bottom_right = (int(width/2) + x, int(height/2) + y)
            
            # Desenha a caixa delimitadora na imagem
            cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)  # Verde, espessura 2

            # Prepara o texto com o nome da classe e a confiança
            label = f"{prediction.class_name}: {prediction.confidence:.5f}"
                        
            # Desenha o texto na imagem
            cv2.putText(cv_image, label, top_left, 
                        cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_color, self.font_thickness, cv2.LINE_AA)
        
        resized_image = cv2.resize(cv_image, (640, 480))
        cv2.imshow('Camera 1', resized_image)
        cv2.waitKey(1)       

    def camera2_callback(self, msg):
        #self.get_logger().info('Recebida imagem da câmera 2')
        # Processamento da imagem da câmera 2
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        predict = self.model.infer(image = cv_image)
        #print(predict)
        for prediction in predict[0].predictions:
            # Extrai as coordenadas e dimensões da caixa delimitadora
            x = int(prediction.x)
            y = int(prediction.y)
            width = int(prediction.width)
            height = int(prediction.height)
            self.angle2 = self.angulo_centro(x,y)
            #print('Camera 2 - x: ', x, 'y: ', y, 'angle: ', self.angle2)

            # Calcula as coordenadas dos cantos da caixa
            top_left = (abs(int(width/2) - x), abs(int(height/2) - y))
            bottom_right = (int(width/2) + x, int(height/2) + y)
            
            # Desenha a caixa delimitadora na imagem
            cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)  # Verde, espessura 2

            # Prepara o texto com o nome da classe e a confiança
            label = f"{prediction.class_name}: {prediction.confidence:.5f}"
                        
            # Desenha o texto na imagem
            cv2.putText(cv_image, label, top_left, 
                        cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_color, self.font_thickness, cv2.LINE_AA)
        
        resized_image = cv2.resize(cv_image, (640, 480))
        cv2.imshow('Camera 2', resized_image)
        cv2.waitKey(1) 

    def camera3_callback(self, msg):
        #self.get_logger().info('Recebida imagem da câmera 3')
        # Processamento da imagem da câmera 3
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        predict = self.model.infer(image = cv_image)
        #print(predict)
        for prediction in predict[0].predictions:
            # Extrai as coordenadas e dimensões da caixa delimitadora
            x = int(prediction.x)
            y = int(prediction.y)
            width = int(prediction.width)
            height = int(prediction.height)
            self.angle3 = self.angulo_centro(x,y)
            #print('Camera 3 - x: ', x, 'y: ', y, 'angle: ', self.angle3)

            # Calcula as coordenadas dos cantos da caixa
            top_left = (abs(int(width/2) - x), abs(int(height/2) - y))
            bottom_right = (int(width/2) + x, int(height/2) + y)
            
            # Desenha a caixa delimitadora na imagem
            cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)  # Verde, espessura 2

            # Prepara o texto com o nome da classe e a confiança
            label = f"{prediction.class_name}: {prediction.confidence:.5f}"
                        
            # Desenha o texto na imagem
            cv2.putText(cv_image, label, top_left, 
                        cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_color, self.font_thickness, cv2.LINE_AA)
        
        resized_image = cv2.resize(cv_image, (640, 480))
        cv2.imshow('Camera 3', resized_image)
        cv2.waitKey(1) 

    def camera4_callback(self, msg):
        #self.get_logger().info('Recebida imagem da câmera 4')
        # Processamento da imagem da câmera 4
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        predict = self.model.infer(image = cv_image)
        #print(predict)
        for prediction in predict[0].predictions:
            
            # Extrai as coordenadas e dimensões da caixa delimitadora
            x = int(prediction.x)
            y = int(prediction.y)
            width = int(prediction.width)
            height = int(prediction.height)
            self.angle4 = self.angulo_centro(x,y)
            #print('Camera 4 - x: ', x, 'y: ', y, 'angle: ', self.angle4)            
            # Calcula as coordenadas dos cantos da caixa
            top_left = (abs(int(width/2) - x), abs(int(height/2) - y))
            bottom_right = (int(width/2) + x, int(height/2) + y)
            
            # Desenha a caixa delimitadora na imagem
            cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)  # Verde, espessura 2
            
            # Prepara o texto com o nome da classe e a confiança
            label = f"{prediction.class_name}: {prediction.confidence:.5f}"
                        
            # Desenha o texto na imagem
            cv2.putText(cv_image, label, top_left, 
                        cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_color, self.font_thickness, cv2.LINE_AA)
        
        resized_image = cv2.resize(cv_image, (640, 480))
        cv2.imshow('Camera 4', resized_image)
        cv2.waitKey(1) 

def main(args=None):
    rclpy.init(args=args)
    node = MultiCamera()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()

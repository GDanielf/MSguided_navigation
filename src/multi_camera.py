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

        # Subscrições para os tópicos de imagem
        
        self.subscription1 = self.create_subscription(Image, '/world/empty/model/rgbd_camera/link/link/sensor/camera_sensor/image', self.camera0_callback, 10)
        self.subscription2 = self.create_subscription(Image, '/world/empty/model/rgbd_camera_1/link/link_1/sensor/camera_sensor_1/image', self.camera1_callback, 10)
        self.subscription3 = self.create_subscription(Image, '/world/empty/model/rgbd_camera_2/link/link_2/sensor/camera_sensor_2/image', self.camera2_callback, 10)
        self.subscription4 = self.create_subscription(Image, '/world/empty/model/rgbd_camera_3/link/link_3/sensor/camera_sensor_3/image', self.camera3_callback, 10)
        self.subscription5 = self.create_subscription(Image, '/world/empty/model/rgbd_camera_4/link/link_4/sensor/camera_sensor_4/image', self.camera4_callback, 10)
        self.subscription6 = self.create_subscription(Image, '/world/empty/model/rgbd_camera_5/link/link_5/sensor/camera_sensor_5/image', self.camera5_callback, 10)
        self.subscription7 = self.create_subscription(Image, '/world/empty/model/rgbd_camera_6/link/link_6/sensor/camera_sensor_6/image', self.camera6_callback, 10)
        self.subscription8 = self.create_subscription(Image, '/world/empty/model/rgbd_camera_7/link/link_7/sensor/camera_sensor_7/image', self.camera7_callback, 10)
        self.subscription9 = self.create_subscription(Image, '/world/empty/model/rgbd_camera_8/link/link_8/sensor/camera_sensor_8/image', self.camera8_callback, 10)
        self.subscription10 = self.create_subscription(Image, '/world/empty/model/rgbd_camera_9/link/link_9/sensor/camera_sensor_9/image', self.camera9_callback, 10)
        
        self.subscription1
        self.subscription2
        self.subscription3
        self.subscription4
        self.subscription5
        self.subscription6
        self.subscription7
        self.subscription8
        self.subscription9
        self.subscription10
        
        # angulos de cada camera para enviar (float)
        self.angle0 = float('nan')
        self.angle1 = float('nan')
        self.angle2 = float('nan')
        self.angle3 = float('nan')
        self.angle4 = float('nan')
        self.angle5 = float('nan')
        self.angle6 = float('nan')
        self.angle7 = float('nan')
        self.angle8 = float('nan')
        self.angle9 = float('nan')    

    def timer_callback(self):
        msg = ImagesAngles()
        msg.angle_image_0 = float(self.angle0)
        msg.angle_image_1 = float(self.angle1)
        msg.angle_image_2 = float(self.angle2)
        msg.angle_image_3 = float(self.angle3)
        msg.angle_image_4 = float(self.angle4)
        msg.angle_image_5 = float(self.angle5)
        msg.angle_image_6 = float(self.angle6)
        msg.angle_image_7 = float(self.angle7)
        msg.angle_image_8 = float(self.angle8)
        msg.angle_image_9 = float(self.angle9)
        self.publisher.publish(msg)
        #self.get_logger().info(f'Published angles: {msg.angle_image_0}, {msg.angle_image_1}, {msg.angle_image_2}, {msg.angle_image_3}')

    
    #funcao que recebe o ponto do centro do retangulo e retorna o angulo entre a reta que divide metade da imagem o ponto 

    def angulo_centro(self,x,y):
        angle = 0
        tang = (x - 959)/(1080 - y)
        angle = math.atan(tang)
        return angle

    def camera0_callback(self, msg):
        #self.get_logger().info('Recebida imagem da câmera 0')
        # Processamento da imagem da câmera 1
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')        
        #pts = np.array([[0, 0], [0, 270], [959, 170], [959,0]], np.int32)
        #pts = pts.reshape((-1, 1, 2))
        #cv2.fillPoly(cv_image, [pts], (0, 0, 0))
        #cv2.rectangle(cv_image, (xb1, yb1), (xb2, yb2), (0, 0, 0), -1)  # -1 preenche o retângulo
        predict = self.model.infer(image = cv_image)
        #tampar parte do corredor para a camera nao ver        
        
        #print('Camera 0 prediction: ', predict)
        if not predict[0].predictions:
            self.angle0 = float('nan')       
        else:
            for prediction in predict[0].predictions:
                # Extrai as coordenadas e dimensões da caixa delimitadora
                #print('Camera 0 prediction: ', prediction)
                x = int(prediction.x)
                y = int(prediction.y)
                width = int(prediction.width)
                height = int(prediction.height)
                self.angle0 = self.angulo_centro(x,y)
                #print('Camera 0 - x: ', x, 'y: ', y, 'angle: ', self.angle0)

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
            
        resized_image = cv2.resize(cv_image, (self.image_width, self.image_height))
        #desenha linha no meio
        cv2.line(resized_image, self.start_point, self.end_point, self.line_color, self.thickness)
        cv2.imshow('Camera 0', resized_image)
        cv2.waitKey(1)   

    def camera1_callback(self, msg):
        #self.get_logger().info('Recebida imagem da câmera 1')
        # Processamento da imagem da câmera 2
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        #xb1, yb1 = 959,0
        #xb2, yb2 = 1919, 270
        #cv2.rectangle(cv_image, (xb1, yb1), (xb2, yb2), (0, 0, 0), -1)  # -1 preenche o retângulo
        predict = self.model.infer(image = cv_image)
        #print(predict)
        if not predict[0].predictions:
            self.angle1 = float('nan')
        else:
            for prediction in predict[0].predictions:
                # Extrai as coordenadas e dimensões da caixa delimitadora
                x = int(prediction.x)
                y = int(prediction.y)
                width = int(prediction.width)
                height = int(prediction.height)
                self.angle1 = self.angulo_centro(x,y)
                #print('Camera 2 - x: ', x, 'y: ', y, 'angle: ', self.angle1)

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
            

        resized_image = cv2.resize(cv_image, (self.image_width, self.image_height))
        #desenha linha no meio
        cv2.line(resized_image, self.start_point, self.end_point, self.line_color, self.thickness)
        cv2.imshow('Camera 1', resized_image)
        cv2.waitKey(1) 

    def camera2_callback(self, msg):
        #self.get_logger().info('Recebida imagem da câmera 2')
        # Processamento da imagem da câmera 3
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        #xb1, yb1 = 959,0
        #xb2, yb2 = 1919, 270
        #cv2.rectangle(cv_image, (xb1, yb1), (xb2, yb2), (0, 0, 0), -1)  # -1 preenche o retângulo
        predict = self.model.infer(image = cv_image)
        #print(predict)
        if not predict[0].predictions:
            self.angle2 = float('nan')
        else:
            for prediction in predict[0].predictions:
                # Extrai as coordenadas e dimensões da caixa delimitadora
                x = int(prediction.x)
                y = int(prediction.y)
                width = int(prediction.width)
                height = int(prediction.height)
                self.angle2 = self.angulo_centro(x,y)
                #print('Camera 3 - x: ', x, 'y: ', y, 'angle: ', self.angle2)

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
                
        resized_image = cv2.resize(cv_image, (self.image_width, self.image_height))
        #desenha linha no meio
        cv2.line(resized_image, self.start_point, self.end_point, self.line_color, self.thickness)
        cv2.imshow('Camera 2', resized_image)
        cv2.waitKey(1) 

    def camera3_callback(self, msg):
        #self.get_logger().info('Recebida imagem da câmera 3')
        # Processamento da imagem da câmera 4
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        #xb1, yb1 = 0,0
        #xb2, yb2 = 959, 270
        #cv2.rectangle(cv_image, (xb1, yb1), (xb2, yb2), (0, 0, 0), -1) 
        predict = self.model.infer(image = cv_image)
        #print(predict)
        if not predict[0].predictions:
            self.angle3 = float('nan')
        else: 
            for prediction in predict[0].predictions:
                # Extrai as coordenadas e dimensões da caixa delimitadora
                x = int(prediction.x)
                y = int(prediction.y)
                width = int(prediction.width)
                height = int(prediction.height)
                self.angle3 = self.angulo_centro(x,y)
                #print('Camera 4 - x: ', x, 'y: ', y, 'angle: ', self.angle3)            
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

        resized_image = cv2.resize(cv_image, (self.image_width, self.image_height))
        #desenha linha no meio
        cv2.line(resized_image, self.start_point, self.end_point, self.line_color, self.thickness)
        cv2.imshow('Camera 3', resized_image)
        cv2.waitKey(1) 

    def camera4_callback(self, msg):
        #self.get_logger().info('Recebida imagem da câmera 4')
        # Processamento da imagem da câmera 4
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        predict = self.model.infer(image = cv_image)
        #print(predict)
        if not predict[0].predictions:
            self.angle4 = float('nan')
        else: 
            for prediction in predict[0].predictions:
                # Extrai as coordenadas e dimensões da caixa delimitadora
                x = int(prediction.x)
                y = int(prediction.y)
                width = int(prediction.width)
                height = int(prediction.height)
                self.angle4 = self.angulo_centro(x,y)
                #print('Camera 4 - x: ', x, 'y: ', y, 'angle: ', self.angle3)            
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

        resized_image = cv2.resize(cv_image, (self.image_width, self.image_height))
        #desenha linha no meio
        cv2.line(resized_image, self.start_point, self.end_point, self.line_color, self.thickness)
        cv2.imshow('Camera 4', resized_image)
        cv2.waitKey(1) 

    def camera5_callback(self, msg):
        #self.get_logger().info('Recebida imagem da câmera 4')
        # Processamento da imagem da câmera 4
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        predict = self.model.infer(image = cv_image)
        #print(predict)
        if not predict[0].predictions:
            self.angle5 = float('nan')
        else: 
            for prediction in predict[0].predictions:
                # Extrai as coordenadas e dimensões da caixa delimitadora
                x = int(prediction.x)
                y = int(prediction.y)
                width = int(prediction.width)
                height = int(prediction.height)
                self.angle5 = self.angulo_centro(x,y)
                #print('Camera 4 - x: ', x, 'y: ', y, 'angle: ', self.angle3)            
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
                
        resized_image = cv2.resize(cv_image, (self.image_width, self.image_height))
        #desenha linha no meio
        cv2.line(resized_image, self.start_point, self.end_point, self.line_color, self.thickness)
        cv2.imshow('Camera 5', resized_image)
        cv2.waitKey(1) 

    def camera6_callback(self, msg):
        #self.get_logger().info('Recebida imagem da câmera 4')
        # Processamento da imagem da câmera 4
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        predict = self.model.infer(image = cv_image)
        #print(predict)
        if not predict[0].predictions:
            self.angle6 = float('nan')
        else: 
            for prediction in predict[0].predictions:
                # Extrai as coordenadas e dimensões da caixa delimitadora
                x = int(prediction.x)
                y = int(prediction.y)
                width = int(prediction.width)
                height = int(prediction.height)
                self.angle6 = self.angulo_centro(x,y)
                #print('Camera 4 - x: ', x, 'y: ', y, 'angle: ', self.angle3)            
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
                
        resized_image = cv2.resize(cv_image, (self.image_width, self.image_height))
        #desenha linha no meio
        cv2.line(resized_image, self.start_point, self.end_point, self.line_color, self.thickness)
        cv2.imshow('Camera 6', resized_image)
        cv2.waitKey(1) 

    def camera7_callback(self, msg):
        #self.get_logger().info('Recebida imagem da câmera 4')
        # Processamento da imagem da câmera 4
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        predict = self.model.infer(image = cv_image)
        #print(predict)
        if not predict[0].predictions:
            self.angle7 = float('nan')
        else: 
            for prediction in predict[0].predictions:
                # Extrai as coordenadas e dimensões da caixa delimitadora
                x = int(prediction.x)
                y = int(prediction.y)
                width = int(prediction.width)
                height = int(prediction.height)
                self.angle7 = self.angulo_centro(x,y)
                #print('Camera 4 - x: ', x, 'y: ', y, 'angle: ', self.angle3)            
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
                
        resized_image = cv2.resize(cv_image, (self.image_width, self.image_height))
        #desenha linha no meio
        cv2.line(resized_image, self.start_point, self.end_point, self.line_color, self.thickness)
        cv2.imshow('Camera 7', resized_image)
        cv2.waitKey(1) 
    
    def camera8_callback(self, msg):
        #self.get_logger().info('Recebida imagem da câmera 4')
        # Processamento da imagem da câmera 4
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        predict = self.model.infer(image = cv_image)
        #print(predict)
        if not predict[0].predictions:
            self.angle8 = float('nan')
        else: 
            for prediction in predict[0].predictions:
                # Extrai as coordenadas e dimensões da caixa delimitadora
                x = int(prediction.x)
                y = int(prediction.y)
                width = int(prediction.width)
                height = int(prediction.height)
                self.angle8 = self.angulo_centro(x,y)
                #print('Camera 4 - x: ', x, 'y: ', y, 'angle: ', self.angle3)            
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
                
        resized_image = cv2.resize(cv_image, (self.image_width, self.image_height))
        #desenha linha no meio
        cv2.line(resized_image, self.start_point, self.end_point, self.line_color, self.thickness)
        cv2.imshow('Camera 8', resized_image)
        cv2.waitKey(1) 

    def camera9_callback(self, msg):
        #self.get_logger().info('Recebida imagem da câmera 4')
        # Processamento da imagem da câmera 4
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        predict = self.model.infer(image = cv_image)
        #print(predict)
        if not predict[0].predictions:
            self.angle9 = float('nan')
        else: 
            for prediction in predict[0].predictions:
                # Extrai as coordenadas e dimensões da caixa delimitadora
                x = int(prediction.x)
                y = int(prediction.y)
                width = int(prediction.width)
                height = int(prediction.height)
                self.angle9 = self.angulo_centro(x,y)
                #print('Camera 4 - x: ', x, 'y: ', y, 'angle: ', self.angle3)            
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
                
        resized_image = cv2.resize(cv_image, (self.image_width, self.image_height))
        #desenha linha no meio
        cv2.line(resized_image, self.start_point, self.end_point, self.line_color, self.thickness)
        cv2.imshow('Camera 9', resized_image)
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

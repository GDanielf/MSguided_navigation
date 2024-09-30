#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
import cv2
import inference
import math


#from guided_navigation.msg import Rectangle
class StereoCamera(Node):
    def __init__(self):
        super().__init__('stereo_camera')

        self.image = None
        self.bridge = CvBridge()  
        self.left_sub = Subscriber(self, Image, '/world/empty/model/stereo_camera/link/left_camera_link/sensor/left_camera/image')
        self.right_sub = Subscriber(self, Image, '/world/empty/model/stereo_camera/link/right_camera_link/sensor/right_camera/image')
        self.ts = ApproximateTimeSynchronizer([self.left_sub, self.right_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.stereo_callback)

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

    def stereo_callback(self, left_image, right_image):
        left_cv_image = self.bridge.imgmsg_to_cv2(left_image, 'bgr8')
        right_cv_image = self.bridge.imgmsg_to_cv2(right_image, 'bgr8')
        predict_left = self.model.infer(image = left_cv_image)
        predict_right = self.model.infer(image = right_cv_image)
        if predict_left[0].predictions:
            for prediction in predict_left[0].predictions:
                # Extrai as coordenadas e dimensões da caixa delimitadora
                x_left = int(prediction.x)
                y_left = int(prediction.y)
                width = int(prediction.width)
                height = int(prediction.height)
                #print('Camera 2 - x: ', x, 'y: ', y, 'angle: ', self.angle1)

                # Calcula as coordenadas dos cantos da caixa
                top_left = (abs(int(width/2) - x_left), abs(int(height/2) - y_left))
                bottom_right = (int(width/2) + x_left, int(height/2) + y_left)
                
                # Desenha a caixa delimitadora na imagem
                cv2.rectangle(left_cv_image, top_left, bottom_right, (0, 255, 0), 2)  # Verde, espessura 2

                # Prepara o texto com o nome da classe e a confiança
                label = f"{prediction.class_name}: {prediction.confidence:.5f}"
                            
                # Desenha o texto na imagem
                cv2.putText(left_cv_image, label, top_left, 
                            cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_color, self.font_thickness, cv2.LINE_AA)
        if predict_right[0].predictions:
            for prediction in predict_right[0].predictions:
                # Extrai as coordenadas e dimensões da caixa delimitadora
                x_right = int(prediction.x)
                y_right = int(prediction.y)
                width = int(prediction.width)
                height = int(prediction.height)

                # Calcula as coordenadas dos cantos da caixa
                top_left = (abs(int(width/2) - x_right), abs(int(height/2) - y_right))
                bottom_right = (int(width/2) + x_right, int(height/2) + y_right)
                
                # Desenha a caixa delimitadora na imagem
                cv2.rectangle(right_cv_image, top_left, bottom_right, (0, 255, 0), 2)  # Verde, espessura 2

                # Prepara o texto com o nome da classe e a confiança
                label = f"{prediction.class_name}: {prediction.confidence:.5f}"
                            
                # Desenha o texto na imagem
                cv2.putText(right_cv_image, label, top_left, 
                            cv2.FONT_HERSHEY_SIMPLEX, self.font_size, self.font_color, self.font_thickness, cv2.LINE_AA)
                
        left_resized_image = cv2.resize(left_cv_image, (self.image_width, self.image_height))
        cv2.imshow('Camera Left', left_resized_image)
        right_resized_image = cv2.resize(right_cv_image, (self.image_width, self.image_height))
        cv2.imshow('Camera Right', right_resized_image)
        cv2.waitKey(1) 
        

def main(args=None):
    rclpy.init(args=args)
    node = StereoCamera()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()

from math import *
import random

class Particle:
    def __init__(self, ruido_virar, sigma_atualizacao, sigma_translacao, tamanho = 30.0):
        self.tamanho = tamanho
        self.x = random.uniform(-tamanho, tamanho)
        self.y = random.uniform(-tamanho, tamanho) 
        self.yaw = random.random() * 2.0 * pi 
        self.ruido_virar = ruido_virar 
        self.sigma_atualizacao = sigma_atualizacao
        self.sigma_translacao = sigma_translacao
        self.w = 0.0

    def move(self, virar, frente):
        new_yaw = (self.yaw + virar + random.gauss(0, self.ruido_virar)) % (2 * pi)  
        dist = frente + random.gauss(0, self.sigma_translacao)
        new_x = (self.x + dist * cos(new_yaw)) 
        new_y = (self.y + dist * sin(new_yaw))         
        self.x = new_x
        self.y = new_y
        self.yaw = new_yaw

    def gaussian(self, mu, sigma, x):
        return exp(-((mu-x)**2)/(sigma**2) / 2) / sqrt(2*pi*(sigma**2))
        
    def measurement_prob(self, pose_est):
        dist = sqrt((pose_est[0] - self.x) ** 2 + (pose_est[1] - self.y) ** 2)  
        self.w = self.gaussian(0, self.sigma_atualizacao, dist)        
    
    def __repr__(self): 
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), 
                                                str(self.yaw))
    
    

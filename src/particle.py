from math import *
import random

class Particle:

    # --------
    # init: 
    #    creates robot and initializes location/yaw 
    #

    def __init__(self, ruido_virar, sigma_atualizacao, sigma_translacao, tamanho = 30.0):
        self.tamanho = tamanho
        self.x = random.uniform(-tamanho, tamanho)
        self.y = random.uniform(-tamanho, tamanho) # initial y position
        self.yaw = random.random() * 2.0 * pi # initial yaw
        self.ruido_virar = ruido_virar # initialize steering noise to zero
        self.sigma_atualizacao = sigma_atualizacao
        self.sigma_translacao = sigma_translacao
        self.w = 0.0

    # --------
    # set: 
    #    sets a robot coordinate    #


    # --------
    # measurement_prob
    #    computes the probability of a measurement
    #  

    def move(self, virar):
        # Adicionando ruído à velocidade linear e angular   
        # Robo indo para frente
        new_yaw = (self.yaw + virar + random.gauss(0, self.ruido_virar)) % (2 * pi)  
        dist = random.gauss(0, self.sigma_translacao)
        new_x = (self.x + dist * cos(new_yaw)) % self.tamanho
        new_y = (self.y + dist * sin(new_yaw)) % self.tamanho        
        self.x = new_x
        self.y = new_y
        self.yaw = new_yaw

    def gaussian(self, mu, sigma, x):
        return exp(-((mu-x)**2)/(sigma**2) / 2) / sqrt(2*pi*(sigma**2))
        
    def measurement_prob(self, pose_est):         
        
        dist = sqrt((pose_est[0] - self.x) ** 2 + (pose_est[1] - self.y) ** 2)     
        
        # update Gaussian
        prob = self.gaussian(0, self.sigma_atualizacao, dist)
        #print(prob)
        self.w = prob
    
    def __repr__(self): #allows us to print particle attributes.
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), 
                                                str(self.yaw))
    
    ############# ONLY ADD/MODIFY CODE BELOW HERE ###################
       
    # --------
    # move: 
    #   
    
    # copy your code from the previous exercise
    # and modify it so that it simulates motion noise
    # according to the noise parameters
    #           self.ruido_virar
    #           self.erro_pose_est
    
    
